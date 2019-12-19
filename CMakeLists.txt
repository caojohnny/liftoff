cmake_minimum_required(VERSION 3.13)
project(liftoff VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 98)

add_subdirectory(liftoff-physics)
add_subdirectory(liftoff-cli)
