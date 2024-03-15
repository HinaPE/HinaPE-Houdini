cmake_minimum_required(VERSION 3.26)
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
project(HinaPE-Houdini LANGUAGES CXX C CUDA)
set(CMAKE_CXX_STANDARD 17)

# Find Houdini
if (MSVC)
    set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.625")
elseif (APPLE)
    set(Houdini_PATH "/Applications/Houdini/Houdini20.0.625/Frameworks/Houdini.framework/Versions/20.0/Resources")
endif ()
set(Houdini_DIR ${Houdini_PATH}/toolkit/cmake)
find_package(Houdini REQUIRED)

# Find OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    link_libraries(OpenMP::OpenMP_CXX)
endif()

# Find CUDA
find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)
if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75)
endif ()
set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)
set(CUDA_SEPARABLE_COMPILATION ON)
set(CUDA_STANDARD 17)

add_subdirectory(HinaPE)
add_subdirectory(hdk)
