cmake_minimum_required(VERSION 3.14)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5Core)

add_executable(MainKinematic
  main.cpp
  my_math.h
  my_math.cpp
)
target_link_libraries(MainKinematic Qt${QT_VERSION_MAJOR}::Core)
