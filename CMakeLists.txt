cmake_minimum_required(VERSION 3.14)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(/home/devel/Projects/SimulatorClient/)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#set(CMAKE_BUILD_TYPE Release)
find_package(Qt5Test REQUIRED)
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
)

target_link_libraries(MainKinematic Qt5::Core Qt5::Test)
