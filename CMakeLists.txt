cmake_minimum_required(VERSION 3.5)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(/home/devel/Projects/SimulatorClient/)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

find_package(Qt5Test)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

#find_package(GTest REQUIRED)

find_package(CUDA REQUIRED)

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

#add_library(QCustomPlot SHARED IMPORTED)
#set_property(TARGET QCustomPlot PROPERTY IMPORTED_LOCATION "")
add_executable(MainKinematic
    main.h
    my_math.h
    robo.h
    test_my_math.h
    cuda_math.h
    emulator.h
    mainwindow.h

    main.cpp
    my_math.cpp
    robo.cpp
    test_my_math.cpp
    some_trash.cpp
    emulator.cpp
    mainwindow.cpp
    ${CUDA_OBJECTS}

    mainwindow.ui
)

target_link_libraries(MainKinematic Qt5::Core Qt5Test cudart Qt5::Widgets#[[QCustomPlot ]] -lqcustomplot#[[${GTest}]])
