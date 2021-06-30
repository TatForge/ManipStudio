cmake_minimum_required(VERSION 3.11)

project(ManipPhysics LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
#include_directories("/home/devel/Projects/MainKinematic/")

set(FILE_COPY_FROM_MAIN TRUE)
if (FILE_COPY_FROM_MAIN)
    configure_file(${CMAKE_SOURCE_DIR}/../physicslib.h 		${CMAKE_SOURCE_DIR}/physicslib.h        COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../physicslib.cpp 	${CMAKE_SOURCE_DIR}/physicslib.cpp 	    COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../solver.h 		    ${CMAKE_SOURCE_DIR}/solver.h 		    COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../solver.cpp 		${CMAKE_SOURCE_DIR}/solver.cpp 		    COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../robo.h            ${CMAKE_SOURCE_DIR}/robo.h              COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../robo.cpp          ${CMAKE_SOURCE_DIR}/robo.cpp            COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../emulator.h 		${CMAKE_SOURCE_DIR}/emulator.h          COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../emulator.cpp 		${CMAKE_SOURCE_DIR}/emulator.cpp        COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../my_math.h 		${CMAKE_SOURCE_DIR}/my_math.h           COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../my_math.cpp 		${CMAKE_SOURCE_DIR}/my_math.cpp         COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../meshcollider.cpp 	${CMAKE_SOURCE_DIR}/meshcollider.cpp 	COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../constants.h		${CMAKE_SOURCE_DIR}/constants.h         COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../my_math.cpp 		${CMAKE_SOURCE_DIR}/my_math.cpp         COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../sharedmem.h       ${CMAKE_SOURCE_DIR}/sharedmem.h         COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../meshcollider.h    ${CMAKE_SOURCE_DIR}/meshcollider.h      COPYONLY)
    configure_file(${CMAKE_SOURCE_DIR}/../meshcollider.cpp  ${CMAKE_SOURCE_DIR}/meshcollider.cpp    COPYONLY)

endif()


add_library(ManipPhysics SHARED
    physicslib.h
    physicslib.cpp
    solver.h
    solver.cpp
    robo.h
    robo.cpp
    emulator.h
    emulator.cpp
    my_math.h
    my_math.cpp
    meshcollider.cpp
    constants.h)
target_link_libraries(ManipPhysics PRIVATE Qt5::Widgets)

#configure_file(${CMAKE_SOURCE_DIR}/../physicslib.h ${CMAKE_SOURCE_DIR}/build/physicslib.h COPYONLY)

add_custom_command(TARGET ManipPhysics POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy 
	${CMAKE_SOURCE_DIR}/build/libManipPhysics.so ${CMAKE_SOURCE_DIR})

	
