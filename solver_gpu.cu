// solver_gpu.cu
#include "solver_gpu.hpp"
#include <cuda_runtime.h>
#include <iostream>
#include <cmath>

// GPU kernel for pairwise collision resolution (naive O(N^2) parallel)
__global__ void kernel_checkCollisions(VerletObject* objs, size_t count, float response)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (int)count) return;

    VerletObject& obj1 = objs[i];

    for (int k = i + 1; k < (int)count; ++k)
    {
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

            float delta = 0.5f * response * (dist - min_dist);

            obj1.position.x -= nx * (mass2 * delta);
            obj1.position.y -= ny * (mass2 * delta);

            obj2.position.x += nx * (mass1 * delta);
            obj2.position.y += ny * (mass1 * delta);
        }
    }
}

__global__ void kernel_applyGravity(VerletObject* objs, size_t n, float gx, float gy)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    objs[i].acceleration.x += gx;
    objs[i].acceleration.y += gy;
}

__global__ void kernel_integrate(VerletObject* objs, size_t n, float dt)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

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

__global__ void kernel_applyConstraint(
    VerletObject* objs, size_t n,
    float cx, float cy, float radius)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    float dx = objs[i].position.x - cx;
    float dy = objs[i].position.y - cy;

    float dist = sqrtf(dx*dx + dy*dy);

    float maxDist = radius - objs[i].radius;

    if (dist > maxDist)
    {
        float nx = dx / dist;
        float ny = dy / dist;

        objs[i].position.x = cx + nx * maxDist;
        objs[i].position.y = cy + ny * maxDist;
    }
}

// --------------------------------------------------
// Constructor
// --------------------------------------------------
SolverGPU::SolverGPU()
{
    d_objects = nullptr;
    d_capacity = 0;
    std::cout << "[GPU] SolverGPU constructed\n";
}

// --------------------------------------------------
// Destructor
// --------------------------------------------------
SolverGPU::~SolverGPU()
{
    if (d_objects)
        cudaFree(d_objects);

    std::cout << "[GPU] SolverGPU destroyed\n";
}

// --------------------------------------------------
// Add object
// --------------------------------------------------
VerletObject& SolverGPU::addObject(sf::Vector2f pos, float radius)
{
    m_objects.emplace_back(pos, radius);
    size_t newCount = m_objects.size();

    if (newCount > d_capacity)
    {
        if (d_objects)
            cudaFree(d_objects);

        d_capacity = newCount * 2;
        cudaMalloc(&d_objects, d_capacity * sizeof(VerletObject));
    }

    // Upload entire array (simple, not optimal)
    cudaMemcpy(d_objects, m_objects.data(),
               newCount * sizeof(VerletObject),
               cudaMemcpyHostToDevice);

    return m_objects.back();
}

// --------------------------------------------------
// Update
// --------------------------------------------------
void SolverGPU::update()
{
    m_time += m_frame_dt;
    const float dt = getStepDt();

    size_t N = m_objects.size();
    if (N == 0) return;

    cudaMemcpy(d_objects, m_objects.data(),
               N * sizeof(VerletObject),
               cudaMemcpyHostToDevice);

    int blockSize = 128;
    int gridSize = (int)((N + blockSize - 1) / blockSize);

    for (uint32_t s = 0; s < m_sub_steps; s++)
    {
        // Gravity
        kernel_applyGravity<<<gridSize, blockSize>>>(
            d_objects, N, m_gravity.x, m_gravity.y);

        // Collisions
        kernel_checkCollisions<<<gridSize, blockSize>>>(
            d_objects, N, 0.75f);

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

    cudaMemcpy(m_objects.data(), d_objects,
               N * sizeof(VerletObject),
               cudaMemcpyDeviceToHost);
}

