// src/gpu/test_gpu.cu
#include <cuda_runtime.h>
#include <iostream>

__global__ void testKernel(int* out) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    out[i] = i * 2;
}

extern "C" void gpu_test() {
    int* d;
    int* h = new int[4];

    cudaMalloc(&d, 4 * sizeof(int));
    testKernel<<<1,4>>>(d);
    cudaMemcpy(h, d, 4*sizeof(int), cudaMemcpyDeviceToHost);

    std::cout << "GPU test output: ";
    for(int i = 0; i < 4; ++i)
        std::cout << h[i] << " ";
    std::cout << std::endl;

    cudaFree(d);
    delete[] h;
}
