cmake_minimum_required(VERSION 3.11)

project(MainKinematic LANGUAGES CXX)
include(FetchContent)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(${CMAKE_SOURCE_DIR}/Tests/models/)
include_directories(${CMAKE_SOURCE_DIR}/STLmodels/)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)
SET(CUDA_REQUIRED false)

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
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
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
    stl_reader.h
    sharedmem.h
    Tests
    Tests/test_my_math.h
    Benchmarks/benchmarks.h

    main.cpp
    my_math.cpp
    robo.cpp
    some_trash.cpp
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
target_link_libraries(MainKinematic Qt5::Core Qt5Test Qt5::Widgets benchmark::benchmark benchmark::benchmark_main GTest::gtest)
target_link_libraries(MainKinematic -lqcustomplot)


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
