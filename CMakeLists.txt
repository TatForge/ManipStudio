cmake_minimum_required(VERSION 3.11)

project(MainKinematic LANGUAGES CXX)
include(FetchContent)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(/home/devel/Projects/SimulatorClient/)
include_directories(/home/devel/Projects/Visualization/VisualizatorOpenGL/)
include_directories(../MainKinematic/Tests/models/)
include_directories(./STLmodels/)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

find_package(Qt5Test)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(benchmark REQUIRED)
find_package(CUDA REQUIRED)
find_package(GTest REQUIRED)
enable_testing(true)

#FetchContent_Declare(
#    gtest
#    GIT_REPOSITORY https://github.com/google/googletest.git
#    GIT TAG release-1.10.0
#    )
##FetchContent_GetProperties(gtest)
##if (NOT gtest_POPULATED)
##    FetchContent_Populate(gtest)
##    add_subdirectory(${gtest_SOURCE_DIR} ${gtest_BINARY_DIR})
##endif()
#FetchContent_MakeAvailable(gtest)

##FetchContent_GetProperties(gtest)
##if (NOT ${gtest}_POPULATED)
##    FetchContent_Populate(gtest)
##    add_subdirectory()
##endif()


include_directories(${CUDA_INCLUDE_DIRS} ${CUDART_INCLUDE_DIRS}
    ${QTEST_INCLUDE_DIRS}  #[[${GTEST_INCLUDE_DIRS}]]#[[${Qt5Test_INCLUDE_DIRS}]])
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
#find_library(CUDA_LIB lib "-L/opt/cuda/lib64")
#if (NOT CUDA_LIB)
#    message("cuda lib not found in /opt/cuda/lib64 \ntrying to find in /usr/local/cuda-10.2/lib64")
#    find_library(CUDA_LIB /usr/local/cuda-10.2/lib64)
#endif()

set(CMAKE_CUDA_ARCHITECTURES 50)
set(CMAKE_CXX_FLAGS "-L/opt/cuda/lib64")

file(GLOB CUDA_FILES "./" *.cu)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} /opt/cuda/targets/x86_64-linux/include)
CUDA_COMPILE(CUDA_OBJECTS ${CUDA_FILES})

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

add_executable(MainKinematic
    main.h
    my_math.h
    robo.h
    Tests/test_my_math.h
    Benchmarks/benchmarks.h
    cuda_math.h
    emulator.h
    mainwindow.h
    solver.h
    constants.h
#    gjkcollision3d.h
    meshcollider.h
    stlreader.h
    stl_reader.h

    main.cpp
    my_math.cpp
    robo.cpp
    Tests/test_my_math.cpp
    Tests/gtest_example.cpp
    Benchmarks/benchmarks.cpp
    some_trash.cpp
    emulator.cpp
    mainwindow.cpp
    solver.cpp
#    gjkcollision3d.cpp
    stlreader.cpp
    meshcollider.cpp
    ${CUDA_OBJECTS}

    mainwindow.ui
)
target_link_libraries(MainKinematic Qt5::Core Qt5Test cudart Qt5::Widgets -lqcustomplot benchmark::benchmark benchmark::benchmark_main GTest::gtest)


#add_executable(UnitTests Tests/gtest_example.cpp)
#target_link_libraries(UnitTests ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
