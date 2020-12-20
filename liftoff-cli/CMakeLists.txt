cmake_minimum_required(VERSION 3.13)
project(liftoff-cli VERSION 1.0 LANGUAGES CXX)

set(MODULE_DIR "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(PARENT_DIR "${MODULE_DIR}" DIRECTORY)
include("${PARENT_DIR}/cmake/DownloadJson.cmake")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PARENT_DIR}/cmake/")
find_package(MathGL2 REQUIRED FLTK)
find_package(FLTK)

add_executable(liftoff-cli
        main.cpp
        recording_fdb.cpp recording_fdb.h
        thrust.cpp thrust.h
        telemetry_flight_profile.cpp telemetry_flight_profile.h
        rocket.cpp rocket.h
        recording_vdb.cpp recording_vdb.h
        pidf_controller.cpp pidf_controller.h
        velocity_flight_profile.cpp velocity_flight_profile.h
        c11_binary_latch.cpp c11_binary_latch.h)
target_include_directories(liftoff-cli
        PRIVATE ${MATHGL2_INCLUDE_DIRS}
        PRIVATE ${FLTK_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(liftoff-cli
        PRIVATE liftoff-physics
        PRIVATE json
        PRIVATE ${MATHGL2_LIBRARIES}
        PRIVATE ${MATHGL2_FLTK_LIBRARIES}
        PRIVATE ${FLTK_LIBRARIES}
        PRIVATE pthread)
