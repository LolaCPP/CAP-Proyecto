#include "gpu_solver.hpp"
#include <cuda_runtime.h>
#include <iostream>

__global__ void kernel_update(ObjectGPU* objects, int N, float dt) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float x = objects[i].x;
    float y = objects[i].y;

    float last_x = objects[i].last_x;
    float last_y = objects[i].last_y;

    // Verlet integration simplificada
    float new_x = x + (x - last_x);
    float new_y = y + (y - last_y) - 9.81f * dt * dt; // gravedad

    objects[i].last_x = x;
    objects[i].last_y = y;

    objects[i].x = new_x;
    objects[i].y = new_y;
}

GPUSolver::GPUSolver(int max_objects)
    : max_objects(max_objects), num_objects(0)
{
    cudaMalloc(&d_objects, max_objects * sizeof(ObjectGPU));
    h_objects = new ObjectGPU[max_objects];
}

GPUSolver::~GPUSolver() {
    cudaFree(d_objects);
    delete[] h_objects;
}

void GPUSolver::createObject(float x, float y) {
    h_objects[num_objects].x = x;
    h_objects[num_objects].y = y;
    h_objects[num_objects].last_x = x - 0.1f;
    h_objects[num_objects].last_y = y;
    num_objects++;
}

void GPUSolver::update(float dt) {
    cudaMemcpy(d_objects, h_objects, num_objects * sizeof(ObjectGPU), cudaMemcpyHostToDevice);

    int blockSize = 256;
    int gridSize = (num_objects + blockSize - 1) / blockSize;

    kernel_update<<<gridSize, blockSize>>>(d_objects, num_objects, dt);

    cudaMemcpy(h_objects, d_objects, num_objects * sizeof(ObjectGPU), cudaMemcpyDeviceToHost);
}
