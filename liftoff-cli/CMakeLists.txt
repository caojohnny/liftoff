cmake_minimum_required(VERSION 3.13)
project(liftoff-cli VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

set(MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(PARENT_DIR "${MODULE_DIR}" DIRECTORY)
include(${PARENT_DIR}/cmake/DownloadMatplotlib.cmake)

add_executable(liftoff-cli
        main.cpp recording_fdb.cpp recording_fdb.h)
target_link_libraries(liftoff-cli
        PRIVATE liftoff-physics
        PRIVATE mpl)
