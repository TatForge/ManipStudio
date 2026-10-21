cmake_minimum_required(VERSION 3.11)

project(ManipPhysics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Core REQUIRED)

# The library is built straight from the main project's src/ and include/ -- there are
# no copies in this directory, so it always matches the tested code.
get_filename_component(MANIP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

add_library(ManipPhysics SHARED
    ${MANIP_SRC_DIR}/include/physicslib.h
    ${MANIP_SRC_DIR}/src/physicslib.cpp
    ${MANIP_SRC_DIR}/include/solver.h
    ${MANIP_SRC_DIR}/src/solver.cpp
    ${MANIP_SRC_DIR}/include/robo.h
    ${MANIP_SRC_DIR}/src/robo.cpp
    ${MANIP_SRC_DIR}/include/emulator.h
    ${MANIP_SRC_DIR}/src/emulator.cpp
    ${MANIP_SRC_DIR}/include/my_math.h
    ${MANIP_SRC_DIR}/src/my_math.cpp
    ${MANIP_SRC_DIR}/include/meshcollider.h
    ${MANIP_SRC_DIR}/src/meshcollider.cpp
    ${MANIP_SRC_DIR}/include/armshape.h
    ${MANIP_SRC_DIR}/src/armshape.cpp
    ${MANIP_SRC_DIR}/include/sharedmem.h
    ${MANIP_SRC_DIR}/src/sharedmem.cpp
    ${MANIP_SRC_DIR}/include/constants.h)
target_include_directories(ManipPhysics PUBLIC ${MANIP_SRC_DIR}/include)
target_link_libraries(ManipPhysics PRIVATE Qt5::Core)

# Keep the ready-to-use library next to this file, as before.
add_custom_command(TARGET ManipPhysics POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:ManipPhysics> ${CMAKE_CURRENT_SOURCE_DIR})
