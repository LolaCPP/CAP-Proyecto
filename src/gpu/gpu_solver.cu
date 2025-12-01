#include "gpu_solver.hpp"
#include <cuda_runtime.h>
#include <cmath>
#include <iostream>

// ----------- Device-side helpers -------------

__device__ int clampi(int v, int minv, int maxv)
{
    if (v < minv) return minv;
    if (v > maxv) return maxv;
    return v;
}

// Clear grid cells on GPU
__global__ void kernel_clear_grid(CollisionCellGPU* grid, int cell_count)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= cell_count) return;

    grid[i].objects_count = 0;
    // objects[] contents don't matter when count = 0
}

// Insert each object into its collision cell (capacity 4, same as CPU)
__global__ void kernel_insert_objects(
    ObjectGPU*        objs,
    int               N,
    CollisionCellGPU* grid,
    int               grid_w,
    int               grid_h
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float x = objs[i].x;
    float y = objs[i].y;

    // Clamp to [1, width-2] / [1, height-2] similar to CPU safety border
    int cx = clampi((int)x, 1, grid_w  - 2);
    int cy = clampi((int)y, 1, grid_h - 2);

    int cell_index = cx * grid_h + cy;

    // Atomic increment to add this object into the cell
    uint32_t idx = atomicAdd(&grid[cell_index].objects_count, 1u);

    // If overflow, ignore (same semantics as fixed capacity)
    if (idx < 4u) {
        grid[cell_index].objects[idx] = (uint32_t)i;
    }
}

// Contact resolution (very close to CPU version)
__device__ void solve_contact(ObjectGPU& obj_1, ObjectGPU& obj_2)
{
    constexpr float response_coef = 1.0f;
    constexpr float eps = 0.0001f;

    float dx = obj_1.x - obj_2.x;
    float dy = obj_1.y - obj_2.y;
    float dist2 = dx * dx + dy * dy;

    if (dist2 < 1.0f && dist2 > eps)
    {
        float dist = sqrtf(dist2);
        float delta = response_coef * 0.5f * (1.0f - dist);
        float nx = dx / dist;
        float ny = dy / dist;

        obj_1.x += nx * delta;
        obj_1.y += ny * delta;
        obj_2.x -= nx * delta;
        obj_2.y -= ny * delta;
    }
}

// For each cell, check collisions with its 8 neighbors + itself
__global__ void kernel_collisions(
    ObjectGPU*        objs,
    CollisionCellGPU* grid,
    int               grid_w,
    int               grid_h
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int cell_count = grid_w * grid_h;
    if (idx >= cell_count) return;

    CollisionCellGPU cell = grid[idx];

    // Offsets for 3x3 neighborhood (same pattern as CPU):
    // index ±1, ±grid_h, combinations
    const int nbh[9] = {
        -grid_h - 1, -grid_h, -grid_h + 1,
        -1,            0,        +1,
        +grid_h - 1, +grid_h, +grid_h + 1
    };

    for (uint32_t i = 0; i < cell.objects_count; ++i)
    {
        uint32_t atom_idx = cell.objects[i];

        for (int ni = 0; ni < 9; ++ni)
        {
            int nidx = idx + nbh[ni];
            if (nidx < 0 || nidx >= cell_count) continue;

            CollisionCellGPU ncell = grid[nidx];

            for (uint32_t j = 0; j < ncell.objects_count; ++j)
            {
                uint32_t atom_2_idx = ncell.objects[j];
                if (atom_2_idx == atom_idx) continue;

                // Solve contact between objs[atom_idx] and objs[atom_2_idx]
                solve_contact(objs[atom_idx], objs[atom_2_idx]);
            }
        }
    }
}

// Verlet integration + gravity + world borders (approximate CPU behavior)
__global__ void kernel_verlet(
    ObjectGPU* objs,
    int        N,
    float      dt,
    float      world_w,
    float      world_h
) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float x  = objs[i].x;
    float y  = objs[i].y;
    float lx = objs[i].last_x;
    float ly = objs[i].last_y;

    // Simple Verlet: newPos = pos + (pos - last_pos) + gravity*dt²
    const float gravity_y = 20.0f;  // matches PhysicSolver's gravity = {0, 20}
    float vx = x - lx;
    float vy = y - ly;

    float nx = x + vx;                    // + acceleration x≈0
    float ny = y + vy + gravity_y * dt * dt;

    // World borders (margin = 2 like CPU)
    const float margin = 2.0f;
    if (nx > world_w - margin) nx = world_w - margin;
    if (nx < margin)           nx = margin;
    if (ny > world_h - margin) ny = world_h - margin;
    if (ny < margin)           ny = margin;

    objs[i].last_x = x;
    objs[i].last_y = y;
    objs[i].x = nx;
    objs[i].y = ny;
}

// --------------- GPUSolver implementation ----------------

GPUSolver::GPUSolver(int max_objects_, int grid_w_, int grid_h_, float world_w_, float world_h_)
    : max_objects(max_objects_),
      num_objects(0),
      grid_w(grid_w_),
      grid_h(grid_h_),
      world_w(world_w_),
      world_h(world_h_)
{
    // Allocate device buffers
    cudaError_t err;

    err = cudaMalloc(&d_objects, max_objects * sizeof(ObjectGPU));
    if (err != cudaSuccess) {
        std::cerr << "cudaMalloc d_objects failed: " << cudaGetErrorString(err) << std::endl;
    }

    int cell_count = grid_w * grid_h;
    err = cudaMalloc(&d_grid, cell_count * sizeof(CollisionCellGPU));
    if (err != cudaSuccess) {
        std::cerr << "cudaMalloc d_grid failed: " << cudaGetErrorString(err) << std::endl;
    }

    // Allocate host mirror
    h_objects = new ObjectGPU[max_objects];
}

GPUSolver::~GPUSolver()
{
    if (d_objects) {
        cudaFree(d_objects);
        d_objects = nullptr;
    }
    if (d_grid) {
        cudaFree(d_grid);
        d_grid = nullptr;
    }
    delete[] h_objects;
    h_objects = nullptr;
}

void GPUSolver::createObject(float x, float y)
{
    if (num_objects >= max_objects) {
        std::cerr << "GPUSolver::createObject: max_objects reached\n";
        return;
    }

    ObjectGPU obj{};
    obj.x      = x;
    obj.y      = y;
    obj.last_x = x - 0.1f;   // similar to your previous initialization
    obj.last_y = y;

    h_objects[num_objects] = obj;

    // Initialize the corresponding GPU slot
    cudaMemcpy(d_objects + num_objects, &obj, sizeof(ObjectGPU), cudaMemcpyHostToDevice);

    num_objects++;
}

void GPUSolver::update_full(float dt)
{
    if (num_objects == 0) return;

    const int blockSize = 256;

    // 1) Clear grid
    int cell_count   = grid_w * grid_h;
    int gridBlocks   = (cell_count + blockSize - 1) / blockSize;
    kernel_clear_grid<<<gridBlocks, blockSize>>>(d_grid, cell_count);

    // 2) Insert objects into grid
    int objBlocks = (num_objects + blockSize - 1) / blockSize;
    kernel_insert_objects<<<objBlocks, blockSize>>>(
        d_objects, num_objects,
        d_grid, grid_w, grid_h
    );

    // 3) Collisions
    kernel_collisions<<<gridBlocks, blockSize>>>(
        d_objects, d_grid,
        grid_w, grid_h
    );

    // 4) Integrate (Verlet + gravity + bounds)
    kernel_verlet<<<objBlocks, blockSize>>>(
        d_objects, num_objects,
        dt,
        world_w, world_h
    );

    // 5) Copy back to host so CPU can read positions if needed
    cudaMemcpy(h_objects, d_objects, num_objects * sizeof(ObjectGPU), cudaMemcpyDeviceToHost);
}

