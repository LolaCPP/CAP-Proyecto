// solver_gpu.hpp
#pragma once
#include <vector>
#include <SFML/Graphics.hpp>

#include "solver_base.hpp"
#include "solver.hpp"  // for VerletObject definition

class SolverGPU : public SolverBase
{
public:
    SolverGPU();
    ~SolverGPU();

    // Implement SolverBase interface
    VerletObject& addObject(sf::Vector2f position, float radius) override;
    void update() override;

    void setSimulationUpdateRate(uint32_t rate) override
    {
        m_frame_dt = 1.0f / static_cast<float>(rate);
    }

    void setConstraint(sf::Vector2f position, float radius) override
    {
        m_constraint_center = position;
        m_constraint_radius = radius;
    }

    void setSubStepsCount(uint32_t sub_steps) override { m_sub_steps = sub_steps; }

    void setObjectVelocity(VerletObject& object, sf::Vector2f v) override
    {
        object.setVelocity(v, getStepDt());
    }

    const std::vector<VerletObject>& getObjects() const override { return m_objects; }

    sf::Vector3f getConstraint() const override
    {
        return {m_constraint_center.x, m_constraint_center.y, m_constraint_radius};
    }

    uint64_t getObjectsCount() const override { return m_objects.size(); }

    float getTime() const override { return m_time; }

    float getStepDt() const override
    {
        return m_frame_dt / static_cast<float>(m_sub_steps);
    }

private:
    uint32_t                  m_sub_steps          = 1;
    sf::Vector2f              m_gravity            = {0.0f, 1000.0f};
    sf::Vector2f              m_constraint_center;
    float                     m_constraint_radius  = 100.0f;

    float                     m_time               = 0.0f;
    float                     m_frame_dt           = 0.0f;

    // CPU copy for renderer
    std::vector<VerletObject> m_objects;

    // GPU data (device buffers)
    VerletObject* d_objects = nullptr;
    size_t d_capacity = 0;
};
