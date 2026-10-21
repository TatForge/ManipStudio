# The kinematics / dynamics core as one static library, shared by the console
# check, the test suites and ManipStudio. It only needs QtCore (QSettings,
# QSharedMemory) and builds with Qt 6 or Qt 5: pass the Qt core target.
#   include(ManipCore.cmake)
#   manip_add_core(Qt6::Core)
function(manip_add_core qt_core_target)
    if(TARGET manip_core)
        return()
    endif()
    set(root ${CMAKE_CURRENT_FUNCTION_LIST_DIR})
    add_library(manip_core STATIC
        ${root}/include/constants.h
        ${root}/include/my_math.h
        ${root}/src/my_math.cpp
        ${root}/include/meshcollider.h
        ${root}/src/meshcollider.cpp
        ${root}/include/armshape.h
        ${root}/src/armshape.cpp
        ${root}/include/robo.h
        ${root}/src/robo.cpp
        ${root}/include/solver.h
        ${root}/src/solver.cpp
        ${root}/include/emulator.h
        ${root}/src/emulator.cpp
        ${root}/include/sharedmem.h
        ${root}/src/sharedmem.cpp)
    target_include_directories(manip_core PUBLIC ${root}/include ${root}/STLmodels)
    target_link_libraries(manip_core PUBLIC ${qt_core_target})
    target_compile_features(manip_core PUBLIC cxx_std_17)
    set_target_properties(manip_core PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
endfunction()
