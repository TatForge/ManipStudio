rm -rf build
mkdir build
cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Release ../
cd build
cmake --build .