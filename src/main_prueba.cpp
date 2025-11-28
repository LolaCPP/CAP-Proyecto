#include <iostream>
#include <chrono>
#include <cstdlib>

#include "physics/physics.hpp"
#include "thread_pool/thread_pool.hpp"

int main(int argc, char** argv)
{
    // 1. Leer número de objetos como argumento
    int N = 2000; // default
    int NUM_THREADS = 1; // default
    if (argc > 1) {
        N = std::atoi(argv[1]);
        NUM_THREADS = std::atoi(argv[2]);
    }

    std::cout << "Creando " << N << " objetos...\n";

    // 2. Crear solver sin SFML
    const IVec2 world_size{300, 300};

    tp::ThreadPool thread_pool(NUM_THREADS);

    PhysicSolver solver(world_size, thread_pool);

    // 3. Crear partículas iniciales
    for (int i = 0; i < N; i++) {
        float x = 10.0f + (i % 100) * 2.0f;
        float y = 10.0f + (i / 100) * 2.0f;

        solver.createObject({x, y});
    }

    // 4. Simular y medir FPS
    const int frames = 400;
    const float dt = 1.0f / 60.0f;

    auto start = std::chrono::high_resolution_clock::now();

    for (int f = 0; f < frames; f++) {
        solver.update(dt);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double total_ms =
        std::chrono::duration<double, std::milli>(end - start).count();

    double fps = frames / (total_ms / 1000.0);

    std::cout << "Simulación completada.\n";
    std::cout << "Frames: " << frames << "\n";
    std::cout << "Tiempo total: " << total_ms << " ms\n";
    std::cout << "FPS promedio: " << fps << "\n";

    return 0;
}
