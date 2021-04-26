cmake_minimum_required(VERSION 3.11)

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
SET(CUDA_REQUIRED false)

#EXECUTE_PROCESS(COMMAND umane -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
#try_run(COMMAND umane -m OUTPUT_VARIABLE ARCHITECTURE)
#message(STATUS ${ARCHITECTURE} "Architecture: ${ARCHITECTURE}")

find_package(Qt5Test)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(benchmark REQUIRED)
find_package(CUDA QUIET)
find_package(GTest REQUIRED)

enable_testing(true)

#if (GTest_FOUND)

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

if (CUDA_FOUND AND CUDA_REQUIRED)
    include_directories(${CUDA_INCLUDE_DIRS} ${CUDART_INCLUDE_DIRS}
        ${QTEST_INCLUDE_DIRS})
    set(CMAKE_CUDA_ARCHITECTURES 50)
    set(CMAKE_CXX_FLAGS "-L/opt/cuda/lib64")
    file(GLOB CUDA_FILES "./" *.cu)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR} /opt/cuda/targets/x86_64-linux/include)
    CUDA_COMPILE(CUDA_OBJECTS ${CUDA_FILES})
    target_link_libraries(MainKinematic cudart)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

#Main target
add_executable(MainKinematic
    main.h
    my_math.h
    robo.h
#    cuda_math.h
    emulator.h
    mainwindow.h
    solver.h
    constants.h
    meshcollider.h
    stlreader.h
#    stl_reader.h
    sharedmem.h
    Tests
    Tests/test_my_math.h
    Benchmarks/benchmarks.h

    main.cpp
    my_math.cpp
    robo.cpp
#    some_trash.cpp
    emulator.cpp
    mainwindow.cpp
    solver.cpp
    sharedmem.cpp
    stlreader.cpp
    meshcollider.cpp
    ${CUDA_OBJECTS}
    Tests/test_my_math.cpp
    Tests/gtest_example.cpp
    Benchmarks/benchmarks.cpp

    mainwindow.ui
)
target_link_libraries(MainKinematic Qt5::Core Qt5Test benchmark::benchmark benchmark::benchmark_main GTest::gtest)

message(STATUS "Architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "x86_64")
    target_link_libraries(MainKinematic -lqcustomplot  Qt5::Widgets)
endif()

configure_file(${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sharedmem.h ${CMAKE_SOURCE_DIR}/sharedmem.h COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/../Visualization/VisualizatorOpenGL/sharedmem.cpp ${CMAKE_SOURCE_DIR}/sharedmem.cpp COPYONLY)

add_custom_command(TARGET MainKinematic POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
                   ${CMAKE_SOURCE_DIR}/ManipulatorSettigns.ini $<TARGET_FILE_DIR:MainKinematic>/)
add_custom_command(TARGET MainKinematic POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory
                  ${CMAKE_SOURCE_DIR}/STLmodels/ $<TARGET_FILE_DIR:MainKinematic>/STLmodels)
#Test target
#add_executable(
#    Tests
##    ${CMAKE_CURRENT_SOURCE_DIR}/
#    Tests/test_my_math.h
#    Benchmarks/benchmarks.h

#    Tests/test_my_math.cpp
#    Tests/gtest_example.cpp
#    Benchmarks/benchmarks.cpp
#    )
#target_include_directories(Tests )



#add_executable(UnitTests Tests/gtest_example.cpp)
#target_link_libraries(UnitTests ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
