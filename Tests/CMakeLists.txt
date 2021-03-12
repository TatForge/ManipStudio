cmake_minimum_required(VERSION 3.5)

project(GTestProject)

include(ExternalProject)
set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR/Tests})

ExternalProject_Add(
    gtest
    GIT_REPOSITORY https://github.com/google/googletest.git
    SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/googletest-src"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/googletest-build"
    TIMEOUT 10
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION}
    LOG_DOWNLOAD ON
    )

#ExternalProject_Get_Property(gtest source_dir)
#set(GTEST_INCLUDE_DIR ${source_dir}/include CACHE INTERNAL "Path to include folder for GTest")
#set(GTEST_ROOT_DIR ${source_dir}/googletest CACHE INTERNAL "Path to source folder for GTest")
#include_directories(${INCLUDE_DIRECTORIES} ${GTEST_INCLUDE_DIR} ${GTEST_ROOD_DIR})
#message(${GTEST_INCLUDE_DIR})
