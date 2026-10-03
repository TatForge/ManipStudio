cmake_minimum_required(VERSION 3.16)

project(MainKinematic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(MANIP_BUILD_TESTS "Build the gtest suites (ManipTests, ManipSolverTests, PhysicsLibTests)" ON)
option(MANIP_BUILD_STUDIO "Build the ManipStudio GUI (needs Qt 6 with Quick3D and Charts)" ON)
option(MANIP_WITH_CUDA "Build the console check with the CUDA helpers (cuda_math.cu)" OFF)

# The core needs only QtCore; Qt 6 is preferred, Qt 5 still works.
set(MANIP_QT_MAJOR "" CACHE STRING "Qt major version for the core: 6 or 5 (empty: Qt 6 if found)")
if(MANIP_QT_MAJOR)
    set(QT_VERSION_MAJOR ${MANIP_QT_MAJOR})
else()
    find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
endif()
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
message(STATUS "Core built with Qt ${Qt${QT_VERSION_MAJOR}Core_VERSION}")

include(ManipCore.cmake)
manip_add_core(Qt${QT_VERSION_MAJOR}::Core)

# Console check: loads the model, plans one MoveJ, prints a summary
add_executable(MainKinematic main.cpp)
target_link_libraries(MainKinematic PRIVATE manip_core)
if(MANIP_WITH_CUDA)
    enable_language(CUDA)
    set(CMAKE_CUDA_ARCHITECTURES 50)
    target_sources(MainKinematic PRIVATE cuda_math.h cuda_math.cu)
endif()
# the core reads its settings and meshes from the working directory
add_custom_command(TARGET MainKinematic POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/ManipulatorSettigns.ini $<TARGET_FILE_DIR:MainKinematic>/
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/STLmodels/ $<TARGET_FILE_DIR:MainKinematic>/STLmodels)

if(MANIP_BUILD_TESTS)
    enable_testing()
    # an installed GoogleTest only on request: a stray one on PATH (e.g. a
    # Windows Anaconda seen from WSL) may be built for another toolchain
    option(MANIP_SYSTEM_GTEST "Use an installed GoogleTest instead of fetching v1.14" OFF)
    if(MANIP_SYSTEM_GTEST)
        find_package(GTest CONFIG REQUIRED)
    else()
        include(FetchContent)
        FetchContent_Declare(gtest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG        v1.14.0)
        set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(gtest)
        add_library(GTest::gtest ALIAS gtest)
    endif()

    set(MANIP_TEST_DEFINITIONS
        WITH_GTESTS=1
        TEST_MODELS_DIR="${CMAKE_SOURCE_DIR}/Tests/models/"
        TEST_PROJECT_DIR="${CMAKE_SOURCE_DIR}/")

    function(manip_add_test name labels)
        add_executable(${name} ${ARGN} Tests/test_helpers.h Tests/test_main.cpp)
        target_compile_definitions(${name} PRIVATE ${MANIP_TEST_DEFINITIONS})
        target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/Tests ${CMAKE_SOURCE_DIR}/Tests/models)
        target_link_libraries(${name} PRIVATE manip_core GTest::gtest)
        add_test(NAME ${name} COMMAND ${name})
        set_tests_properties(${name} PROPERTIES LABELS "${labels}")
    endfunction()

    # Phase 1-2: math, MeshCollider, Robo
    manip_add_test(ManipTests "math;robo"
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
        Tests/test_arm_shape.cpp
        Tests/test_elem.cpp
        Tests/test_robo.cpp
        Tests/test_robo_behavior.cpp
        Tests/test_robo_dynamics.cpp)

    # Phase 3: Solver (kinematics, trajectories, dynamics, collisions), Emulator, SharedMem
    manip_add_test(ManipSolverTests "solver"
        Tests/solver_test_access.h
        Tests/solver_fixture.h
        Tests/test_sharedmem.cpp
        Tests/test_emulator.cpp
        Tests/test_solver_kinematics.cpp
        Tests/test_solver_trajectories.cpp
        Tests/test_solver_dynamics.cpp
        Tests/test_solver_collisions.cpp)

    # Phase 4: PhysicsLib public API
    manip_add_test(PhysicsLibTests "lib"
        physicslib.h
        physicslib.cpp
        Tests/test_physicslib.cpp)
endif()

if(MANIP_BUILD_STUDIO)
    if(QT_VERSION_MAJOR EQUAL 6)
        find_package(Qt6 6.4 QUIET COMPONENTS Quick Quick3D Charts QuickControls2)
    endif()
    if(Qt6Quick3D_FOUND AND Qt6Charts_FOUND)
        add_subdirectory(Studio)
    else()
        message(STATUS "ManipStudio skipped: needs Qt 6 with Quick, Quick3D and Charts")
    endif()
endif()
