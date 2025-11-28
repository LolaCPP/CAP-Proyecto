mkdir build
cd build
cmake -DSFML_WINDOW=OFF -DSFML_GRAPHICS=OFF -DSFML_AUDIO=OFF ..
cmake --build .