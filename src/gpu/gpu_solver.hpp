#pragma once
#include <vector>

struct ObjectGPU {
    float x, y;
    float last_x, last_y;
};

class GPUSolver {
public:
    GPUSolver(int max_objects);
    ~GPUSolver();

    void createObject(float x, float y);
    void update(float dt);

private:
    int max_objects;
    int num_objects;

    ObjectGPU* d_objects;  // array en GPU
    ObjectGPU* h_objects;  // copia CPU temporal
};
