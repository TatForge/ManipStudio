cmake_minimum_required(VERSION 3.16)

# ManipStudio: the single Qt6/QML window that replaces the old Qt5 windows.
# Standalone project: cmake -S Studio -B build-studio
project(ManipStudio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 6.4 REQUIRED COMPONENTS Core Gui Widgets Qml Quick QuickControls2 Quick3D Charts Test)

get_filename_component(MANIP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

# The kinematics / dynamics core, compiled against Qt6::Core (it only needs
# QSettings and QSharedMemory).
add_library(manip_core STATIC
    ${MANIP_SRC_DIR}/constants.h
    ${MANIP_SRC_DIR}/my_math.h
    ${MANIP_SRC_DIR}/my_math.cpp
    ${MANIP_SRC_DIR}/meshcollider.h
    ${MANIP_SRC_DIR}/meshcollider.cpp
    ${MANIP_SRC_DIR}/robo.h
    ${MANIP_SRC_DIR}/robo.cpp
    ${MANIP_SRC_DIR}/solver.h
    ${MANIP_SRC_DIR}/solver.cpp
    ${MANIP_SRC_DIR}/emulator.h
    ${MANIP_SRC_DIR}/emulator.cpp
    ${MANIP_SRC_DIR}/sharedmem.h
    ${MANIP_SRC_DIR}/sharedmem.cpp)
target_include_directories(manip_core PUBLIC ${MANIP_SRC_DIR} ${MANIP_SRC_DIR}/STLmodels)
target_link_libraries(manip_core PUBLIC Qt6::Core)
set_target_properties(manip_core PROPERTIES AUTOMOC OFF)

# Qt-side bridge to the core (no QML, so it can be tested headless).
add_library(studio_backend STATIC
    solverworker.h
    solverworker.cpp
    meshlayout.h
    meshlayout.cpp
    robotbackend.h
    robotbackend.cpp)
target_include_directories(studio_backend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(studio_backend PUBLIC manip_core Qt6::Core Qt6::Gui Qt6::Charts)

# Directory with ManipulatorSettigns.ini and STLmodels/ used when --data is not given.
set(MANIP_DATA_DIR ${MANIP_SRC_DIR} CACHE PATH "Directory with ManipulatorSettigns.ini and STLmodels/")

qt_add_executable(ManipStudio main.cpp)
set(STUDIO_QML Main.qml Theme.qml Card.qml JointPanel.qml MovePanel.qml PosePanel.qml RobotView.qml AxisLines.qml ResultsPanel.qml PeaksPanel.qml Chip.qml)
set(STUDIO_QML_FILES)
foreach(f ${STUDIO_QML})
    # served at qrc:/Studio/<f>, next to the module's qmldir, so that the
    # files see each other (and the Theme singleton) through the module
    set_source_files_properties(qml/${f} PROPERTIES QT_RESOURCE_ALIAS ${f})
    list(APPEND STUDIO_QML_FILES qml/${f})
endforeach()
# must be set before qt_add_qml_module reads the file list
set_source_files_properties(qml/Theme.qml PROPERTIES QT_QML_SINGLETON_TYPE TRUE)
qt_add_qml_module(ManipStudio
    URI Studio
    VERSION 1.0
    QML_FILES ${STUDIO_QML_FILES}
    SOURCES scenegeometry.h scenegeometry.cpp)
# Qt 6.4's AOT compiler spins forever on singleton lookups (Theme.*); bytecode only.
set_target_properties(ManipStudio PROPERTIES QT_QMLCACHEGEN_ARGUMENTS "--only-bytecode")
target_compile_definitions(ManipStudio PRIVATE MANIP_DATA_DIR="${MANIP_DATA_DIR}")
target_link_libraries(ManipStudio PRIVATE studio_backend
    Qt6::Widgets Qt6::Qml Qt6::Quick Qt6::QuickControls2 Qt6::Quick3D Qt6::Charts)

enable_testing()
add_executable(StudioTests tests/test_robotbackend.cpp)
target_compile_definitions(StudioTests PRIVATE MANIP_DATA_DIR="${MANIP_DATA_DIR}")
target_link_libraries(StudioTests PRIVATE studio_backend Qt6::Test)
add_test(NAME StudioTests COMMAND StudioTests)
set_tests_properties(StudioTests PROPERTIES LABELS "studio" ENVIRONMENT "QT_QPA_PLATFORM=offscreen")

add_executable(StudioSceneTests tests/test_meshlayout.cpp)
target_compile_definitions(StudioSceneTests PRIVATE MANIP_DATA_DIR="${MANIP_DATA_DIR}")
target_link_libraries(StudioSceneTests PRIVATE studio_backend Qt6::Test)
add_test(NAME StudioSceneTests COMMAND StudioSceneTests)
set_tests_properties(StudioSceneTests PROPERTIES LABELS "studio")
