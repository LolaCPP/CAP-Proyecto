# Compile
nvcc -o verlet_app \
    main.cpp \
    -arch=sm_86 \
    -I./ \
    -lsfml-graphics -lsfml-window -lsfml-system \
    -std=c++17 \
    --compiler-bindir=/usr/bin/g++ \
    -Xcompiler "-O2 -Wno-narrowing"
