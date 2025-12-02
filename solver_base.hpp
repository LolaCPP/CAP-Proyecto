// solver_base.hpp
#pragma once

#include <cstdint>
#include <vector>
#include <SFML/Graphics.hpp>

struct VerletObject; // forward declaration

class SolverBase
{
public:
    virtual ~SolverBase() = default;

    // Core simulation API
    virtual void update() = 0;

    virtual VerletObject& addObject(sf::Vector2f position, float radius) = 0;

    virtual void setSimulationUpdateRate(uint32_t rate) = 0;
    virtual void setConstraint(sf::Vector2f position, float radius) = 0;
    virtual void setSubStepsCount(uint32_t sub_steps) = 0;
    virtual void setObjectVelocity(VerletObject& object, sf::Vector2f v) = 0;

    // Read-only access for rendering / UI
    virtual const std::vector<VerletObject>& getObjects() const = 0;
    virtual sf::Vector3f getConstraint() const = 0;
    virtual uint64_t getObjectsCount() const = 0;
    virtual float getTime() const = 0;
    virtual float getStepDt() const = 0;
};
