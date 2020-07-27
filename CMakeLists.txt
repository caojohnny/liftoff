cmake_minimum_required(VERSION 3.13)
project(liftoff VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_subdirectory(liftoff-physics)
add_subdirectory(liftoff-cli)
