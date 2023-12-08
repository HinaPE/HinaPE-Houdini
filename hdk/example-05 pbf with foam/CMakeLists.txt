cmake_minimum_required(VERSION 3.26)
project(example_04_PBF)

set(CMAKE_CXX_STANDARD 17)

set(TARGET PBDClothSolver)

option(BuildWithEigen "Build with Eigen" ON)

# Find Houdini
set(Houdini_PATH "/Applications/Houdini/Houdini20.0.547/Frameworks/Houdini.framework/Versions/20.0/Resources")
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Load Source
add_library(
        ${TARGET}
        SHARED
        Entrance.cpp
        SIM_PBFSolver.cpp
        SIM_PBFSolver.h

        pbf/data.h
        pbf/interface.h
        pbf/solver.cpp
        pbf/solver.h
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
        pbf
)
if (APPLE)
    target_compile_options(
            ${TARGET}
            PRIVATE
            -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
    )
endif ()

if (BuildWithEigen)
    # Find Eigen
    find_package(Eigen3 REQUIRED)
    if (EIGEN3_FOUND)
        message(STATUS "Eigen3 found")
        message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
        target_link_libraries(
                ${TARGET}
                PUBLIC
                Eigen3::Eigen
        )
        add_definitions(-DUSE_EIGEN)
    else ()
        message(FATAL_ERROR "Eigen3 not found")
    endif ()
endif ()

houdini_configure_target(${TARGET})
