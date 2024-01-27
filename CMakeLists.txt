cmake_minimum_required(VERSION 3.26)
project(HinaPE-Houdini)

set(CMAKE_CXX_STANDARD 17)

# Find Houdini
if (MSVC)
    set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.590")
elseif (APPLE)
    set(Houdini_PATH "/Applications/Houdini/Houdini20.0.590/Frameworks/Houdini.framework/Versions/20.0/Resources")
endif ()
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Config TBB only for Windows
if (MSVC)
    include(install/cmake/FindTBB.cmake)
    include(install/cmake/TaskingSystemOptions.cmake)
    include(install/cmake/CompileOptions.cmake)
endif ()

add_subdirectory(hdk)
