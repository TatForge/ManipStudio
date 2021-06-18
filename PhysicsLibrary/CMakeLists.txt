cmake_minimum_required(VERSION 3.11)

project(ManipPhysics LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
include_directories("/home/devel/Projects/MainKinematic/")

add_library(ManipPhysics SHARED
    ../physicslib.h
    ../physicslib.cpp
    ../solver.cpp
    ../robo.h
    ../robo.cpp
    ../emulator.cpp
    ../my_math.cpp
    ../meshcollider.cpp
    ../constants.h)
target_link_libraries(ManipPhysics PRIVATE Qt5::Widgets)

configure_file(${CMAKE_SOURCE_DIR}/../physicslib.h ${CMAKE_SOURCE_DIR}/physicslib.h COPYONLY)

add_custom_command(TARGET ManipPhysics POST_BUILD COMMAND ${CMAKE_COMMAND} - E copy 
	${CMAKE_SOURCE_DIR}/build/libManipPhysics.so ${CMAKE_SOURCE_DIR})
