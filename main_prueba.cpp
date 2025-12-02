// main_prueba.cpp
#include <iostream>
#include <chrono>
#include <cstdlib>

#include "physics/physics.hpp"
#include "thread_pool/thread_pool.hpp"
#include <cuda_runtime.h>

// ---- GPU CHECK ----
void checkGPU()
{
    int count = 0;
    cudaError_t err = cudaGetDeviceCount(&count);

    if (err != cudaSuccess)
    {
        std::cout << "cudaGetDeviceCount ERROR: "
                  << cudaGetErrorString(err) << std::endl;
        return;
    }

    if (count == 0)
    {
        std::cout << "NO CUDA GPU DETECTED!" << std::endl;
        return;
    }

    cudaDeviceProp prop{};
    cudaGetDeviceProperties(&prop, 0);

    std::cout << "GPU DETECTED: " << prop.name << std::endl;
}

// IMPORTANT: match definition in test_gpu.cu
extern "C" void gpu_test();

int main(int argc, char **argv)
{
    // Check GPU and run tiny test kernel
    checkGPU();
    gpu_test();

    // 1. Leer número de objetos como argumento
    int N = 2000;        // default
    int NUM_THREADS = 1; // default
    if (argc > 1)
    {
        N = std::atoi(argv[1]);
        NUM_THREADS = std::atoi(argv[2]);
    }

    // 2. Crear solver sin SFML
    const IVec2 world_size{300, 300};

    tp::ThreadPool thread_pool(NUM_THREADS);

    // argv[3] == 1  -> usar GPU
    bool use_gpu = (argc > 3 ? std::atoi(argv[3]) : 0);

    PhysicSolver solver(world_size, thread_pool, use_gpu);

    // 3. Crear partículas iniciales
    for (int i = 0; i < N; i++)
    {
        float x = 10.0f + (i % 100) * 2.0f;
        float y = 10.0f + (i / 100) * 2.0f;

        solver.createObject({x, y});
    }

    // 4. Simular y medir FPS
    const int frames = 400;
    const float dt = 1.0f / 60.0f;

    auto start = std::chrono::high_resolution_clock::now();

    for (int f = 0; f < frames; f++)
    {
        solver.update(dt);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double total_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    double fps = frames / (total_ms / 1000.0);

    std::cout << "FPS: " << fps << std::endl;

    return 0;
}
