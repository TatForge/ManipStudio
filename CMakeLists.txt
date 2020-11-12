cmake_minimum_required(VERSION 3.14)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5Core)

add_executable(MainKinematic
    main.h
    my_math.h
    robo.h

    main.cpp
    my_math.cpp
    robo.cpp
)
target_link_libraries(MainKinematic Qt${QT_VERSION_MAJOR}::Core)
