cmake_minimum_required(VERSION 3.5)

project(Box2D NONE)

include(ExternalProject)
ExternalProject_Add(Box2D
        GIT_REPOSITORY    https://github.com/tobanteGaming/Box2D-cmake.git
        GIT_TAG           master
        SOURCE_DIR        "${CMAKE_CURRENT_BINARY_DIR}/Box2D-src"
        BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/Box2D-build"
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
        TEST_COMMAND      ""
)