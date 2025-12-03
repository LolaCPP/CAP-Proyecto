// solver_gpu.cu
#include "solver_gpu.hpp"
#include <cuda_runtime.h>
#include <iostream>
#include <cmath>

// Small POD type to accumulate positional corrections per particle
struct Delta2
{
    float x;
    float y;
};

// --------------------------------------------------
// KERNELS
// --------------------------------------------------

__global__ void kernel_applyGravity(VerletObject* objs, size_t n, float gx, float gy)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)n) return;

    objs[i].acceleration.x += gx;
    objs[i].acceleration.y += gy;
}

// Accumulate collision corrections into delta[] using atomics.
// We DO NOT write directly to objs[i].position here to avoid races.
__global__ void kernel_collide(
    VerletObject* objs,
    Delta2*       delta,
    size_t        count,
    float         response)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)count) return;

    const int N = (int)count;

    for (int k = i + 1; k < N; ++k)
    {
        VerletObject& obj1 = objs[i];
        VerletObject& obj2 = objs[k];

        float vx = obj1.position.x - obj2.position.x;
        float vy = obj1.position.y - obj2.position.y;
        float dist2 = vx * vx + vy * vy;

        float min_dist = obj1.radius + obj2.radius;

        if (dist2 < min_dist * min_dist)
        {
            float dist = sqrtf(dist2 + 1e-6f);
            float nx = vx / dist;
            float ny = vy / dist;

            float mass1 = obj1.radius / (obj1.radius + obj2.radius);
            float mass2 = obj2.radius / (obj1.radius + obj2.radius);

            float deltaDist = 0.5f * response * (dist - min_dist);

            // Corrections for each object
            float dx1 = -nx * (mass2 * deltaDist);
            float dy1 = -ny * (mass2 * deltaDist);

            float dx2 =  nx * (mass1 * deltaDist);
            float dy2 =  ny * (mass1 * deltaDist);

            // Accumulate using atomics to avoid races
            atomicAdd(&delta[i].x, dx1);
            atomicAdd(&delta[i].y, dy1);

            atomicAdd(&delta[k].x, dx2);
            atomicAdd(&delta[k].y, dy2);
        }
    }
}

// Apply accumulated collision corrections
__global__ void kernel_applyDelta(VerletObject* objs, const Delta2* delta, size_t n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)n) return;

    objs[i].position.x += delta[i].x;
    objs[i].position.y += delta[i].y;
}

// Clear delta array each substep
__global__ void kernel_clearDelta(Delta2* delta, size_t n)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)n) return;

    delta[i].x = 0.0f;
    delta[i].y = 0.0f;
}

// Circular constraint (same as CPU but per particle)
__global__ void kernel_applyConstraint(
    VerletObject* objs, size_t n,
    float cx, float cy, float radius)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)n) return;

    float dx = objs[i].position.x - cx;
    float dy = objs[i].position.y - cy;

    float dist = sqrtf(dx * dx + dy * dy);
    float maxDist = radius - objs[i].radius;

    if (dist > maxDist)
    {
        float nx = dx / dist;
        float ny = dy / dist;

        objs[i].position.x = cx + nx * maxDist;
        objs[i].position.y = cy + ny * maxDist;
    }
}

// Verlet integration, same math as CPU updateObjects()
__global__ void kernel_integrate(VerletObject* objs, size_t n, float dt)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)n) return;

    float px = objs[i].position.x;
    float py = objs[i].position.y;

    float lx = objs[i].position_last.x;
    float ly = objs[i].position_last.y;

    float ax = objs[i].acceleration.x;
    float ay = objs[i].acceleration.y;

    float nx = px + (px - lx) + ax * dt * dt;
    float ny = py + (py - ly) + ay * dt * dt;

    objs[i].position_last.x = px;
    objs[i].position_last.y = py;

    objs[i].position.x = nx;
    objs[i].position.y = ny;

    objs[i].acceleration.x = 0.0f;
    objs[i].acceleration.y = 0.0f;
}

// --------------------------------------------------
// SolverGPU methods
// --------------------------------------------------

SolverGPU::SolverGPU()
{
    d_objects  = nullptr;
    d_capacity = 0;
    std::cout << "[GPU] SolverGPU constructed\n";
}

SolverGPU::~SolverGPU()
{
    if (d_objects)
        cudaFree(d_objects);

    std::cout << "[GPU] SolverGPU destroyed\n";
}

VerletObject& SolverGPU::addObject(sf::Vector2f pos, float radius)
{
    m_objects.emplace_back(pos, radius);
    size_t newCount = m_objects.size();

    // Grow GPU buffer if needed
    if (newCount > d_capacity)
    {
        if (d_objects)
            cudaFree(d_objects);

        d_capacity = newCount * 2;
        cudaError_t err = cudaMalloc(&d_objects, d_capacity * sizeof(VerletObject));
        if (err != cudaSuccess)
        {
            std::cerr << "[CUDA ERROR] cudaMalloc failed in addObject: "
                      << cudaGetErrorString(err) << "\n";
        }
    }

    // Upload entire array (simple, not optimal but OK for now)
    cudaMemcpy(d_objects, m_objects.data(),
               newCount * sizeof(VerletObject),
               cudaMemcpyHostToDevice);

    return m_objects.back();
}

void SolverGPU::update()
{
    m_time += m_frame_dt;
    const float dt = getStepDt();

    size_t N = m_objects.size();
    if (N == 0) return;

    // Upload CPU → GPU
    cudaMemcpy(d_objects, m_objects.data(),
               N * sizeof(VerletObject),
               cudaMemcpyHostToDevice);

    // Temporary delta buffer for this frame (freed at end)
    Delta2* d_delta = nullptr;
    cudaError_t err = cudaMalloc(&d_delta, N * sizeof(Delta2));
    if (err != cudaSuccess)
    {
        std::cerr << "[CUDA ERROR] cudaMalloc d_delta failed: "
                  << cudaGetErrorString(err) << "\n";
        return;
    }

    int blockSize = 128;
    int gridSize  = (int)((N + blockSize - 1) / blockSize);

    const float response_coef = 0.75f;

    for (uint32_t s = 0; s < m_sub_steps; ++s)
    {
        // Gravity
        kernel_applyGravity<<<gridSize, blockSize>>>(
            d_objects, N, m_gravity.x, m_gravity.y);

        // Clear delta before collisions
        kernel_clearDelta<<<gridSize, blockSize>>>(d_delta, N);

        // Collisions (accumulate corrections)
        kernel_collide<<<gridSize, blockSize>>>(
            d_objects, d_delta, N, response_coef);

        // Apply collision corrections
        kernel_applyDelta<<<gridSize, blockSize>>>(
            d_objects, d_delta, N);

        // Constraints
        kernel_applyConstraint<<<gridSize, blockSize>>>(
            d_objects, N,
            m_constraint_center.x, m_constraint_center.y,
            m_constraint_radius);

        // Integrate
        kernel_integrate<<<gridSize, blockSize>>>(
            d_objects, N, dt);
    }

    cudaDeviceSynchronize();
    err = cudaGetLastError();
    if (err != cudaSuccess)
    {
        std::cerr << "[CUDA ERROR] after update kernels: "
                  << cudaGetErrorString(err) << "\n";
    }

    // Download back to CPU for rendering
    cudaMemcpy(m_objects.data(), d_objects,
               N * sizeof(VerletObject),
               cudaMemcpyDeviceToHost);

    cudaFree(d_delta);
}
