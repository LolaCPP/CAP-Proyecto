// main.cpp
#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>

#include "solver_base.hpp"
#include "solver.hpp"
#include "solver_gpu.hpp"
#include "renderer.hpp"
#include "utils/number_generator.hpp"
#include "utils/math.hpp"

static sf::Color getRainbow(float t)
{
    const float r = std::sin(t);
    const float g = std::sin(t + 0.33f * 2.0f * Math::PI);
    const float b = std::sin(t + 0.66f * 2.0f * Math::PI);
    return {static_cast<uint8_t>(255.0f * r * r),
            static_cast<uint8_t>(255.0f * g * g),
            static_cast<uint8_t>(255.0f * b * b)};
}

int main(int argc, char* argv[])
{
    constexpr int32_t window_width  = 1000;
    constexpr int32_t window_height = 1000;

    sf::ContextSettings settings;
    settings.antialiasingLevel = 1;
    sf::RenderWindow window(sf::VideoMode(window_width, window_height),
                            "Verlet", sf::Style::Default, settings);
    const uint32_t frame_rate = 120;
    window.setFramerateLimit(frame_rate);

    bool useGPU = false;
    if (argc > 1)
        useGPU = (std::stoi(argv[1]) != 0);

    Solver    solverCPU;
    SolverGPU solverGPU;

    SolverBase* solver = useGPU
        ? static_cast<SolverBase*>(&solverGPU)
        : static_cast<SolverBase*>(&solverCPU);

    if (useGPU)
        std::cout << "[MAIN] Using GPU solver\n";
    else
        std::cout << "[MAIN] Using CPU solver\n";

    Renderer renderer{window};

    solver->setConstraint({static_cast<float>(window_width) * 0.5f,
                           static_cast<float>(window_height) * 0.5f},
                          450.0f);
    solver->setSubStepsCount(8);
    solver->setSimulationUpdateRate(frame_rate);

    const float object_spawn_delay   = 1.025f;
    const float object_spawn_speed   = 1200.0f;
    const sf::Vector2f object_spawn_position = {500.0f, 200.0f};
    const float object_min_radius    = 5.0f;
    const float object_max_radius    = 5.0f;
    const uint32_t max_objects_count = 1000;
    const float max_angle            = 1.0f;

    sf::Clock clock;
    sf::Clock fpsClock;
    int frameCount = 0;

    while (window.isOpen())
    {
        sf::Event event{};
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed ||
                sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
            {
                window.close();
            }
        }

        if (solver->getObjectsCount() < max_objects_count &&
            clock.getElapsedTime().asSeconds() >= object_spawn_delay)
        {
            clock.restart();
            auto &object = solver->addObject(
                object_spawn_position,
                RNGf::getRange(object_min_radius, object_max_radius));

            const float t = solver->getTime();
            const float angle = max_angle * std::sin(t) + Math::PI * 0.5f;
            solver->setObjectVelocity(object,
                object_spawn_speed * sf::Vector2f{std::cos(angle), std::sin(angle)});
            object.color = getRainbow(t);
        }

        solver->update();

        window.clear(sf::Color::White);
        renderer.render(*solver);

        frameCount++;

        if (fpsClock.getElapsedTime().asSeconds() >= 1.0f)
        {
            const uint64_t objCount = solver->getObjectsCount();

            window.setTitle(
                std::string("Verlet ")
                + (useGPU ? "[GPU]" : "[CPU]")
                + " | FPS: " + std::to_string(frameCount)
                + " | Objects: " + std::to_string(objCount));

            frameCount = 0;
            fpsClock.restart();
        }

        window.display();
    }

    return 0;
}
