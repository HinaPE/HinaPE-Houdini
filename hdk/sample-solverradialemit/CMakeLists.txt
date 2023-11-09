# This file handles building each task as sub-project.
#
# Each task becomes an independent project in which you can experiment.
#
# Tasks are added as separate sub-projects so as compilation errors in
# one task don't affect another task.

cmake_minimum_required(VERSION 3.26)
project(HinaPE-Houdini)

set(CMAKE_CXX_STANDARD 17)

set(TARGET SolverRadialEmit)

set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.506")
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

set(SRC
        SIM_RadialEmit.h
        SIM_RadialEmit.C
)

add_library(
        ${TARGET}  SHARED ${SRC}
)
target_link_libraries(${TARGET} PUBLIC Houdini)
target_include_directories(${TARGET}
        PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)
houdini_configure_target(${TARGET})
