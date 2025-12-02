// utils/math.hpp
#pragma once
#include <cmath>
#include "../solver.hpp"

struct Math
{
    static constexpr float PI = 3.1415936f;

    static Vec2 dot(Vec2 v1, Vec2 v2)
    {
        return {v1.x * v2.x, v1.y * v2.y};
    }
};