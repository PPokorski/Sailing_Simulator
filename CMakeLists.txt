cmake_minimum_required(VERSION 3.13)
project(sailing_simulator)

set(CMAKE_CXX_STANDARD 14)

configure_file(CMakeLists.txt.in Box2D/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Box2D )
if(result)
    message(FATAL_ERROR "CMake step for Box2D failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Box2D )
if(result)
    message(FATAL_ERROR "Build step for Box2D failed: ${result}")
endif()

add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/Box2D-src
        ${CMAKE_CURRENT_BINARY_DIR}/Box2D-build
        EXCLUDE_FROM_ALL)

set(CMAKE_INCLUDE_CURRENT_DIR  ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5Widgets CONFIG REQUIRED)

find_package(Eigen3 REQUIRED)

add_executable(sailing_simulator main.cpp main_window.cpp motor_boat.cpp)
target_link_libraries(sailing_simulator TG::Box2D Qt5::Widgets Eigen3::Eigen)