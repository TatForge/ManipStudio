cmake_minimum_required(VERSION 3.5)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(/home/devel/Projects/SimulatorClient/)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(Qt5Test)

find_package(CUDA REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS} ${CUDART_INCLUDE_DIRS})

#find_library(CUDA_LIB lib "-L/opt/cuda/lib64")
#if (NOT CUDA_LIB)
#    message("cuda lib not found in /opt/cuda/lib64 \ntrying to find in /usr/local/cuda-10.2/lib64")
#    find_library(CUDA_LIB /usr/local/cuda-10.2/lib64)
#endif()

set(CMAKE_CXX_FLAGS "-L/opt/cuda/lib64")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

add_executable(MainKinematic
    main.h
    my_math.h
    robo.h
    test_my_math.h

    main.cpp
    my_math.cpp
    robo.cpp
    test_my_math.cpp
    cuda_math.cu
)

target_link_libraries(MainKinematic Qt5::Core Qt5::Test cudart)
