cmake_minimum_required(VERSION 3.13)
project(liftoff-cli VERSION 1.0 LANGUAGES CXX)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(PARENT_DIR "${MODULE_DIR}" DIRECTORY)
include("${PARENT_DIR}/cmake/DownloadJson.cmake")

find_package(ROOT REQUIRED)
include(${ROOT_USE_FILE})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PARENT_DIR}/cmake/")
find_package(GMP REQUIRED)

add_library(root INTERFACE)
target_include_directories(root
        INTERFACE "${ROOT_INCLUDE_DIRS}")
target_link_libraries(root
        INTERFACE "${ROOT_LIBRARIES}")

root_generate_dictionary(data_plotter_dict
        data_plotter.h
        LINKDEF data_plotter_link.h)

add_library(data_plotter SHARED data_plotter.cpp data_plotter_dict.cxx)
target_include_directories(data_plotter
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(data_plotter
        PUBLIC root)

add_executable(liftoff-cli
        main.cpp
        recording_fdb.cpp recording_fdb.h
        thrust.cpp thrust.h
        telemetry_flight_profile.cpp telemetry_flight_profile.h
        rocket.cpp rocket.h
        recording_vdb.cpp recording_vdb.h
        pidf_controller.cpp pidf_controller.h
        velocity_flight_profile.cpp velocity_flight_profile.h)
target_link_libraries(liftoff-cli
        PRIVATE liftoff-physics
        PRIVATE data_plotter
        PRIVATE json
        PRIVATE gmpxx
        PRIVATE gmp)
