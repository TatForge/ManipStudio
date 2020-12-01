cmake_minimum_required(VERSION 3.14)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(/home/devel/Projects/SimulatorClient/)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_BUILD_TYPE Release)

add_executable(MainKinematic
    main.h
    my_math.h
    robo.h

    main.cpp
    my_math.cpp
    robo.cpp
)
