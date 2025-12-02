# Compile
nvcc -c solver_gpu.cu -o solver_gpu.o \
    -arch=sm_86 \
    -std=c++17 \
    --compiler-bindir=/usr/bin/gcc-12 \
    -Xcompiler "-O2 -Wno-narrowing" \
    -allow-unsupported-compiler

g++ main.cpp solver_gpu.o -o verlet_app \
    -lsfml-graphics -lsfml-window -lsfml-system \
    -lcudart \
    -L/usr/local/cuda/lib64 \
    -I/usr/local/cuda/include \
    -std=c++17 \
    -lm -O2
