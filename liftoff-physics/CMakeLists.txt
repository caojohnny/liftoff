cmake_minimum_required(VERSION 3.13)
project(liftoff-physics VERSION 1.0 LANGUAGES CXX)

include(GNUInstallDirs)

set(MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(PARENT_DIR "${MODULE_DIR}" DIRECTORY)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PARENT_DIR}/cmake/")
find_package(GMP REQUIRED)

add_library(liftoff-physics
        liftoff-physics/body.cpp liftoff-physics/body.h
        liftoff-physics/drag.cpp liftoff-physics/drag.h
        liftoff-physics/vector.cpp liftoff-physics/vector.h
        liftoff-physics/driven_body.cpp liftoff-physics/driven_body.h
        liftoff-physics/force_driven_body.cpp liftoff-physics/force_driven_body.h
        liftoff-physics/velocity_driven_body.cpp liftoff-physics/velocity_driven_body.h
        liftoff-physics/linalg.cpp liftoff-physics/linalg.h
        liftoff-physics/polynomial.cpp liftoff-physics/polynomial.h
        liftoff-physics/matrix.cpp liftoff-physics/matrix.h liftoff-physics/telem_proc.cpp liftoff-physics/telem_proc.h)
target_include_directories(liftoff-physics
        PUBLIC "$<BUILD_INTERFACE:${MODULE_DIR}>"
        PUBLIC "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(liftoff-physics
        PRIVATE gmpxx
        PRIVATE gmp)

include("${PARENT_DIR}/cmake/ExportLibrary.cmake")
