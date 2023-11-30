cmake_minimum_required(VERSION 3.26)
project(example_03)

set(CMAKE_CXX_STANDARD 17)

set(TARGET PBDClothSolver)

# Find Houdini
set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.506")
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Load Source
add_library(
        ${TARGET}
        SHARED
        Entrance.cpp
        solver.cpp
        solver.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${TARGET}
        PUBLIC
        Houdini
)
target_include_directories(
        ${TARGET}
        PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)
houdini_configure_target(${TARGET})
