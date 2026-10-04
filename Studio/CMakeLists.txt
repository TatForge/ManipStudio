cmake_minimum_required(VERSION 3.16)

# ManipStudio: the Qt6/QML GUI of the project. Built from the top-level
# CMakeLists.txt, or on its own: cmake -S Studio -B build-studio
project(ManipStudio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 6.4 REQUIRED COMPONENTS Core Gui Widgets Qml Quick QuickControls2 Quick3D Charts Test)

get_filename_component(MANIP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

# The kinematics / dynamics core (already defined when built from the top-level project)
include(${MANIP_SRC_DIR}/ManipCore.cmake)
manip_add_core(Qt6::Core)

# Qt-side bridge to the core (no QML, so it can be tested headless).
add_library(studio_backend STATIC
    solverworker.h
    solverworker.cpp
    meshlayout.h
    meshlayout.cpp
    visuallayout.h
    visuallayout.cpp
    robotbackend.h
    robotbackend.cpp
    translation.h
    translation.cpp)
# UI translations, read at run time from :/i18n/studio_<code>.ts (no lrelease needed)
qt_add_resources(studio_backend "studio_i18n" PREFIX "/i18n" BASE i18n FILES i18n/studio_ru.ts)
target_include_directories(studio_backend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(studio_backend PUBLIC manip_core Qt6::Core Qt6::Gui Qt6::Charts)

# Motor set-up over CAN (ODrive). The simulator always builds; real adapters need
# Qt SerialPort (SLCAN USB adapters) and / or Qt SerialBus (Qt CAN plugins).
find_package(Qt6 QUIET COMPONENTS SerialPort SerialBus)
add_library(studio_motors STATIC
    motors/canframe.h
    motors/cantransport.h
    motors/odriveprotocol.h
    motors/odriveprotocol.cpp
    motors/slcan.h
    motors/slcan.cpp
    motors/simulatedodrive.h
    motors/simulatedodrive.cpp
    motors/hardwaretransports.h
    motors/hardwaretransports.cpp
    motors/motorhub.h
    motors/motorhub.cpp)
target_include_directories(studio_motors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/motors)
target_link_libraries(studio_motors PUBLIC Qt6::Core Qt6::Charts)
if(Qt6SerialPort_FOUND)
    target_link_libraries(studio_motors PUBLIC Qt6::SerialPort)
    target_compile_definitions(studio_motors PRIVATE MANIP_HAVE_SERIALPORT)
endif()
if(Qt6SerialBus_FOUND)
    target_link_libraries(studio_motors PUBLIC Qt6::SerialBus)
    target_compile_definitions(studio_motors PRIVATE MANIP_HAVE_SERIALBUS)
endif()
message(STATUS "ManipStudio motors: SLCAN ${Qt6SerialPort_FOUND}, Qt CAN plugins ${Qt6SerialBus_FOUND}")

# Directory with ManipulatorSettigns.ini and STLmodels/ used when --data is not given.
set(MANIP_DATA_DIR ${MANIP_SRC_DIR} CACHE PATH "Directory with ManipulatorSettigns.ini and STLmodels/")

qt_add_executable(ManipStudio main.cpp)
set(STUDIO_QML Main.qml Theme.qml Card.qml JointPanel.qml MovePanel.qml PosePanel.qml RobotView.qml AxisLines.qml ResultsPanel.qml PeaksPanel.qml Chip.qml NumberField.qml SettingsPanel.qml JogPanel.qml ProgramPanel.qml MotorWindow.qml)
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
target_link_libraries(ManipStudio PRIVATE studio_backend studio_motors
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

add_executable(StudioI18nTests tests/test_translation.cpp)
target_compile_definitions(StudioI18nTests PRIVATE MANIP_DATA_DIR="${MANIP_DATA_DIR}" STUDIO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(StudioI18nTests PRIVATE studio_backend Qt6::Test)
add_test(NAME StudioI18nTests COMMAND StudioI18nTests)
set_tests_properties(StudioI18nTests PROPERTIES LABELS "studio")

add_executable(MotorTests tests/test_motors.cpp)
target_link_libraries(MotorTests PRIVATE studio_motors Qt6::Test)
add_test(NAME MotorTests COMMAND MotorTests)
set_tests_properties(MotorTests PROPERTIES LABELS "studio")
