#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "solver.hpp"
#include "utils/number_generator.hpp"
#include "utils/math.hpp"


int main()
{
    const uint32_t frame_rate = 120;
    const float frame_dt = 1.0f / frame_rate;

    Solver solver;

    // Parámetros de simulación
    const float window_width  = 1000.0f;
    const float window_height = 1000.0f;

    solver.setConstraint({window_width * 0.5f, window_height * 0.5f}, 450.0f);
    solver.setSubStepsCount(8);
    solver.setSimulationUpdateRate(frame_rate);

    // Parámetros de spawn
    const float object_spawn_delay  = 0.025f;
    const float object_spawn_speed  = 1200.0f;
    const Vec2  object_spawn_pos    = {500.0f, 200.0f};
    const float object_min_radius   = 5.0f;
    const float object_max_radius   = 5.0f;
    const uint32_t max_objects_count = 1000;
    const float max_angle = 1.0f;

    // Timing usando std::chrono
    auto last_spawn_time = std::chrono::high_resolution_clock::now();
    auto last_fps_time   = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    float elapsed_since_max = 0.0f;
    bool max_reached = false;
    std::chrono::high_resolution_clock::time_point max_reached_time;


    while (true) // bucle hasta llegar al máximo de objetos
    {
        // Spawn automático
        auto now = std::chrono::high_resolution_clock::now();
        float spawn_elapsed = std::chrono::duration<float>(now - last_spawn_time).count();

        if (solver.getObjectsCount() < max_objects_count && spawn_elapsed >= object_spawn_delay)
        {
            last_spawn_time = now;

            float radius = RNGf::getRange(object_min_radius, object_max_radius);
            auto& obj = solver.addObject(object_spawn_pos, radius);

            float t = solver.getTime();
            float angle = max_angle * std::sin(t) + Math::PI * 0.5f;

            solver.setObjectVelocity(obj, { 
                object_spawn_speed * std::cos(angle),
                object_spawn_speed * std::sin(angle)
            });

            if (solver.getObjectsCount() == max_objects_count)
            {
                max_reached = true;
                max_reached_time = now;

                std::cout << "[INFO] Alcanzado máximo (" << max_objects_count 
                          << "). La simulación continuará 10s más.\n";
            }

        }

        // Actualiza física
        solver.update();

        // FPS por consola cada segundo
        frame_count++;
        float fps_elapsed = std::chrono::duration<float>(now - last_fps_time).count();
        if (fps_elapsed >= 1.0f)
        {
            std::cout << "FPS: " << frame_count 
                      << " | Objects: " << solver.getObjectsCount()
                      << "\n";

            frame_count = 0;
            last_fps_time = now;
        }

        // Simula frame rate
        std::this_thread::sleep_for(std::chrono::duration<float>(frame_dt));

        if (max_reached)
        {
            float elapsed_since_max = std::chrono::duration<float>(now - max_reached_time).count();

            if (elapsed_since_max >= 10.0f)
            {
                std::cout << "[INFO] Han pasado 10s desde el máximo. Fin de simulación.\n";
                break;
            }
        }
    }

    return 0;
}
