cmake_minimum_required(VERSION 3.13)
project(sailing_simulator)

set(CMAKE_CXX_STANDARD 14)

include(cmake/ConfigureExternalProject.cmake)

configureExternalProject(Box2D cmake/Box2D.cmake)
configureExternalProject(googletest cmake/GTest.cmake)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_AUTOUIC_SEARCH_PATHS ui)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)

find_package(Eigen3 REQUIRED)

find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)

include_directories(include/)

#set(PHYSICS_SRC
#        src/physics/entity.cpp
#        src/physics/dynamic_object.cpp
#        src/physics/motor_boat.cpp
#        src/physics/object.cpp
#        src/physics/sailing_boat.cpp
#        src/physics/static_object.cpp
#        src/physics/world.cpp)

set(OBJECTS_SRC
        src/objects/body_components/dynamic_body_component.cpp
        src/objects/body_components/static_body_component.cpp
        src/objects/dynamics_components/composite_dynamics_component.cpp
        src/objects/dynamics_components/engine_dynamics_component.cpp
        src/objects/dynamics_components/rudder_dynamics_component.cpp
        src/objects/dynamics_components/wind_dynamics_component.cpp
        include/sailing_simulator/objects/input_components/qt_motor_boat_input_component.hpp
        src/objects/input_components/qt_motor_boat_input_component.cpp
        src/objects/world.cpp src/objects/input_components/qt_sailing_boat_input_component.cpp include/sailing_simulator/objects/input_components/qt_sailing_boat_input_component.hpp)

add_library(${PROJECT_NAME}_lib
        ${OBJECTS_SRC})

target_link_libraries(${PROJECT_NAME}_lib
        tobanteGaming::Box2D
        Eigen3::Eigen
        Qt5::Core
        Qt5::Widgets)

add_executable(${PROJECT_NAME}
        src/physics_test.cpp
        include/sailing_simulator/debug/main_window.hpp
        src/debug/main_window.cpp)

target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_lib
        tobanteGaming::Box2D
        Qt5::Widgets
        Eigen3::Eigen)

add_executable(${PROJECT_NAME}_test
        test/src/objects/body_components/dynamic_body_component_test.cpp
        test/src/objects/dynamics_components/wind_dynamics_component_helpers_test.cpp
        test/src/objects/dynamics_components/wind_dynamics_component_test.cpp)

include_directories(test/include)

target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}_lib
        Eigen3::Eigen
        tobanteGaming::Box2D
        gtest
        gtest_main)

add_test(NAME ${PROJECT_NAME}_test
         COMMAND ${PROJECT_NAME}_test)