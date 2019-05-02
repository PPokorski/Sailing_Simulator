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

find_package(Qt5Widgets CONFIG REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include/)

set(PHYSICS_SRC
        src/physics/entity.cpp
        src/physics/dynamic_object.cpp
        src/physics/motor_boat.cpp
        src/physics/object.cpp
        src/physics/sailing_boat.cpp
        src/physics/static_object.cpp
        src/physics/world.cpp)

set(OBJECTS_SRC
        src/objects/body_components/dynamic_body_component.cpp
        src/objects/body_components/static_body_component.cpp
        src/objects/dynamics_components/engine_dynamics_component.cpp
        src/objects/dynamics_components/wind_dynamics_component.cpp
        src/objects/world.cpp)

#add_executable(sailing_simulator main.cpp main_window.cpp motor_boat.cpp ${PHYSICS_SRC})
#target_link_libraries(sailing_simulator TG::Box2D Qt5::Widgets Eigen3::Eigen)

add_executable(physics_test
        src/physics_test.cpp
        include/sailing_simulator/debug/main_window.hpp
        src/debug/main_window.cpp
        ${PHYSICS_SRC}
        ${OBJECTS_SRC})
target_link_libraries(physics_test TG::Box2D Qt5::Widgets Eigen3::Eigen)