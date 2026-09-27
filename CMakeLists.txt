cmake_minimum_required(VERSION 3.14)

project(MainKinematic LANGUAGES CXX)
include(FetchContent)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_SOURCE_DIR}/Tests/models/)
include_directories(${CMAKE_SOURCE_DIR}/STLmodels/)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

SET(BUILD_LIB OFF)
SET(CUDA_REQUIRED false)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O0")
#EXECUTE_PROCESS(COMMAND umane -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
#try_run(COMMAND umane -m OUTPUT_VARIABLE ARCHITECTURE)
#message(STATUS ${ARCHITECTURE} "Architecture: ${ARCHITECTURE}")

find_package(Qt5Test)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(benchmark QUIET)
find_package(CUDA QUIET)
message("Cuda dirs = " ${CUDA_INCLUDE_DIRS})
find_package(GTest QUIET)
include_directories(${Qt5Widgets_INCLUDE_DIRS})

enable_testing(true)

FetchContent_Declare(
    gtest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    )
FetchContent_MakeAvailable(gtest)

#Main target
add_executable(MainKinematic
    my_math.h
    robo.h
    emulator.h
    mainwindow.h
    solver.h
    constants.h
    meshcollider.h
    physicslib.h
    stl_reader.h
    sharedmem.h
#    sysvsharedmemory.h

    main.cpp
    my_math.cpp
    robo.cpp
#    some_trash.cpp
    physicslib.cpp
    emulator.cpp
    mainwindow.cpp
    solver.cpp
    sharedmem.cpp
#    sysvsharedmemory.cpp
    meshcollider.cpp

    mainwindow.ui
)
target_link_libraries(MainKinematic Qt5::Core Qt5::Test)
if (CUDA_FOUND AND ${CUDA_REQUIRED})
    message("Cuda found " ${CUDA_FOUND})
    include_directories(${CUDA_INCLUDE_DIRS} ${CUDART_INCLUDE_DIRS}
                        ${QTEST_INCLUDE_DIRS})
    set(CMAKE_CUDA_ARCHITECTURES 50)
    set(CMAKE_CXX_FLAGS "-L/opt/cuda/lib64")
    file(GLOB CUDA_FILES "./" *.cu)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR} /opt/cuda/targets/x86_64-linux/include)
#    CUDA_COMPILE(CUDA_OBJECTS ${CUDA_FILES})
    set_source_files_properties(CUDA_FILES PROPERTIES LANGUAGE CUDA)
    target_sources(MainKinematic PRIVATE cuda_math.h cuda_math.cu)
    target_link_libraries(MainKinematic ${CUDA_LIBRARIES})
endif()

add_custom_command(TARGET MainKinematic POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
               ${CMAKE_SOURCE_DIR}/ManipulatorSettigns.ini $<TARGET_FILE_DIR:MainKinematic>/)
add_custom_command(TARGET MainKinematic POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
              ${CMAKE_SOURCE_DIR}/STLmodels/ $<TARGET_FILE_DIR:MainKinematic>/STLmodels)

#Test targets
# Core sources shared by all test executables (compiled once).
add_library(manip_test_core OBJECT
    my_math.h
    my_math.cpp
    meshcollider.h
    meshcollider.cpp
    robo.h
    robo.cpp
    constants.h
)
# Solver / Emulator / SharedMem layer (needs STLmodels + QSharedMemory).
add_library(manip_test_solver OBJECT
    solver.h
    solver.cpp
    emulator.h
    emulator.cpp
    sharedmem.h
    sharedmem.cpp
)
foreach(obj manip_test_core manip_test_solver)
    target_compile_definitions(${obj} PRIVATE WITH_GTESTS=1)
    target_link_libraries(${obj} PRIVATE Qt5::Core)
endforeach()

set(MANIP_TEST_DEFINITIONS
    WITH_GTESTS=1
    TEST_MODELS_DIR="${CMAKE_SOURCE_DIR}/Tests/models/"
    TEST_PROJECT_DIR="${CMAKE_SOURCE_DIR}/")

# Phase 1-2: math, MeshCollider, Robo
add_executable(ManipTests
    $<TARGET_OBJECTS:manip_test_core>
    Tests/test_helpers.h
    Tests/test_constants.cpp
    Tests/test_rotations.cpp
    Tests/test_vector3.cpp
    Tests/test_vector.cpp
    Tests/test_matrix.cpp
    Tests/test_matrix4x4.cpp
    Tests/test_quaternion.cpp
    Tests/test_tensor.cpp
    Tests/test_angles_statevector.cpp
    Tests/test_polynom.cpp
    Tests/test_math_utils.cpp
    Tests/test_meshcollider.cpp
    Tests/test_elem.cpp
    Tests/test_robo.cpp
    Tests/test_robo_behavior.cpp
    Tests/test_main.cpp
)
target_compile_definitions(ManipTests PRIVATE ${MANIP_TEST_DEFINITIONS})
target_link_libraries(ManipTests PRIVATE gtest Qt5::Core)
add_test(NAME ManipTests COMMAND ManipTests)
set_tests_properties(ManipTests PROPERTIES LABELS "math;robo")

# Phase 3: Solver (kinematics, trajectories, dynamics, collisions), Emulator, SharedMem
add_executable(ManipSolverTests
    $<TARGET_OBJECTS:manip_test_core>
    $<TARGET_OBJECTS:manip_test_solver>
    Tests/test_helpers.h
    Tests/solver_test_access.h
    Tests/solver_fixture.h
    Tests/test_sharedmem.cpp
    Tests/test_emulator.cpp
    Tests/test_solver_kinematics.cpp
    Tests/test_solver_trajectories.cpp
    Tests/test_solver_dynamics.cpp
    Tests/test_solver_collisions.cpp
    Tests/test_main.cpp
)
target_compile_definitions(ManipSolverTests PRIVATE ${MANIP_TEST_DEFINITIONS})
target_link_libraries(ManipSolverTests PRIVATE gtest Qt5::Core)
add_test(NAME ManipSolverTests COMMAND ManipSolverTests)
set_tests_properties(ManipSolverTests PROPERTIES LABELS "solver")

# Phase 4: PhysicsLib public API
add_executable(PhysicsLibTests
    $<TARGET_OBJECTS:manip_test_core>
    $<TARGET_OBJECTS:manip_test_solver>
    physicslib.h
    physicslib.cpp
    Tests/test_helpers.h
    Tests/test_physicslib.cpp
    Tests/test_main.cpp
)
target_compile_definitions(PhysicsLibTests PRIVATE ${MANIP_TEST_DEFINITIONS})
target_link_libraries(PhysicsLibTests PRIVATE gtest Qt5::Core)
add_test(NAME PhysicsLibTests COMMAND PhysicsLibTests)
set_tests_properties(PhysicsLibTests PROPERTIES LABELS "lib")

message(STATUS "Architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    target_link_libraries(MainKinematic -lqcustomplot  Qt5::Widgets)
endif()

if(BUILD_LIB)
    set(FILES_FOR_COPY
        ${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sharedmem.h
        ${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sharedmem.cpp
        ${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sysvsharedmemory.h
        ${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sysvsharedmemory.cpp
        ${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sysvsharedmemory.cpp)

    configure_file(${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sharedmem.h ${CMAKE_SOURCE_DIR}/sharedmem.h COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sharedmem.cpp ${CMAKE_SOURCE_DIR}/sharedmem.cpp COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sysvsharedmemory.h ${CMAKE_SOURCE_DIR}/sysvsharedmemory.h COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sysvsharedmemory.cpp ${CMAKE_SOURCE_DIR}/sysvsharedmemory.cpp COPYONLY)


    add_library(physicslib STATIC
        physicslib.h
        robo.h
        physicslib.cpp
        solver.cpp
        robo.cpp
        emulator.cpp
        my_math.cpp
        meshcollider.cpp)
    target_link_libraries(physicslib PUBLIC Qt5Core Qt5::Widgets)
endif()
