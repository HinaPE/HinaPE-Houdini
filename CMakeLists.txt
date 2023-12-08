cmake_minimum_required(VERSION 3.26)
project(example_03)

set(CMAKE_CXX_STANDARD 17)

set(TARGET PBDClothSolver)

# Find Houdini
set(Houdini_PATH "/Applications/Houdini/Houdini20.0.547/Frameworks/Houdini.framework/Versions/20.0/Resources")
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
target_compile_options(
        ${TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${TARGET})
