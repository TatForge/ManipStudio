# The kinematics / dynamics core as one static library, shared by the console
# check, the test suites and ManipStudio. It only needs QtCore (QSettings,
# QSharedMemory) and builds with Qt 6 or Qt 5: pass the Qt core target.
#   include(ManipCore.cmake)
#   manip_add_core(Qt6::Core)
function(manip_add_core qt_core_target)
    if(TARGET manip_core)
        return()
    endif()
    set(src ${CMAKE_CURRENT_FUNCTION_LIST_DIR})
    add_library(manip_core STATIC
        ${src}/constants.h
        ${src}/my_math.h
        ${src}/my_math.cpp
        ${src}/meshcollider.h
        ${src}/meshcollider.cpp
        ${src}/robo.h
        ${src}/robo.cpp
        ${src}/solver.h
        ${src}/solver.cpp
        ${src}/emulator.h
        ${src}/emulator.cpp
        ${src}/sharedmem.h
        ${src}/sharedmem.cpp)
    target_include_directories(manip_core PUBLIC ${src} ${src}/STLmodels)
    target_link_libraries(manip_core PUBLIC ${qt_core_target})
    target_compile_features(manip_core PUBLIC cxx_std_17)
    set_target_properties(manip_core PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
endfunction()
