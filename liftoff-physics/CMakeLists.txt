cmake_minimum_required(VERSION 3.13)
project(liftoff-physics VERSION 1.0 LANGUAGES CXX)

include(GNUInstallDirs)

set(MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(PARENT_DIR "${MODULE_DIR}" DIRECTORY)

set(CMAKE_CXX_STANDARD 11)

add_library(liftoff-physics
        liftoff-physics/body.cpp liftoff-physics/body.h liftoff-physics/vector.cpp liftoff-physics/vector.h)
target_include_directories(liftoff-physics
        PUBLIC "$<BUILD_INTERFACE:${MODULE_DIR}>"
        PUBLIC "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

include("${PARENT_DIR}/cmake/ExportLibrary.cmake")
