#pragma once
#include <cstdint>

// Simple GPU-side object for Verlet integration
struct ObjectGPU {
    float x, y;
    float last_x, last_y;
};

// GPU-side collision cell, mirroring CPU CollisionCell (capacity = 4)
struct CollisionCellGPU {
    uint32_t objects_count;
    uint32_t objects[4];
};

class GPUSolver {
public:
    GPUSolver(int max_objects, int grid_w, int grid_h, float world_w, float world_h);
    ~GPUSolver();

    // Create a new object on GPU (initial position & velocity)
    void createObject(float x, float y);

    // Full GPU update: build grid, solve collisions, integrate, then copy back to host
    void update_full(float dt);

public:
    int max_objects;
    int num_objects;

    int grid_w;
    int grid_h;

    float world_w;
    float world_h;

    // Device memory
    ObjectGPU*        d_objects   = nullptr;
    CollisionCellGPU* d_grid      = nullptr;

    // Host mirror
    ObjectGPU*        h_objects   = nullptr;
};

