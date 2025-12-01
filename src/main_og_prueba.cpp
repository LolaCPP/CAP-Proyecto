#include <iostream>
#include <chrono>
#include <cstdlib>
#include "physics/physics.hpp"
#include "thread_pool/thread_pool.hpp"


int main(int argc, char** argv)
{
    // Default values
    int N = 2000;
    int NUM_THREADS = 1;

    // Read command line arguments
    if (argc > 1) N = std::atoi(argv[1]);
    if (argc > 2) NUM_THREADS = std::atoi(argv[2]);
    
    // Simulation parameters
    const int frames = 400;
    const float dt = 1.0f / 60.0f;

    // Start timing
    auto start = std::chrono::high_resolution_clock::now();
    
    // Initialize solver and thread pool
    tp::ThreadPool thread_pool(NUM_THREADS);
    const IVec2 world_size{300, 300};
    PhysicSolver solver{world_size, thread_pool};

    // Create initial objects
    while (solver.objects.size() < N)
        solver.createObject({2.0f, 10.0f + 1.1f * solver.objects.size()});

    // Main simulation loop
    for (int f = 0; f < 100; f++)
        solver.update(dt);

    // End timing
    auto end = std::chrono::high_resolution_clock::now();
    double total_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    double fps = frames / (total_ms / 1000.0);

    // Output the fps
    std::cout << fps << std::endl;

    return 0;
}