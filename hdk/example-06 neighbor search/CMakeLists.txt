cmake_minimum_required(VERSION 3.26)
project(example_06)

set(CMAKE_CXX_STANDARD 17)

set(TARGET NeighborSearch)

# Find Houdini
set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.547")
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
        TreeNSearch
)
target_link_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ThirdParty/TreeNSearch/lib
)
target_include_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ThirdParty/TreeNSearch/include
)
houdini_configure_target(${TARGET})
