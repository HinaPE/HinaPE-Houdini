cmake_minimum_required(VERSION 3.26)
project(HinaPE-Houdini)

set(CMAKE_CXX_STANDARD 17)

set(TARGET HinaSolver01)

set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.506")
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

set(SRC
        Entrance.cpp
        solver.h
        solver.cpp
        data.cpp
        data.h
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
