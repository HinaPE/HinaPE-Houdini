cmake_minimum_required(VERSION 3.26)

project(SIM_PBF)

set(CMAKE_CXX_STANDARD 17)
set(TARGET SIM_PBFSolver)

# Find Houdini
set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.547")
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Load Source
add_library(
        ${TARGET}
        SHARED
        Entrance.cpp
        SIM_PBF.cpp
        SIM_PBF.h

        src/solver.cpp
        src/solver.h
        src/utils.cpp
        src/utils.h
        src/data.cpp
        src/data.h
)

# Link Houdini Toolkit
target_link_libraries(
        ${TARGET}
        PUBLIC
        Houdini
)
target_link_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(
        ${TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
houdini_configure_target(${TARGET})

set(TEST_TARGET test)
add_executable(
        ${TEST_TARGET}
        test.cpp
)
# Link Houdini Toolkit
target_link_libraries(
        ${TEST_TARGET}
        PUBLIC
        ${TARGET}
        Houdini
)
target_link_directories(
        ${TEST_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(
        ${TEST_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
