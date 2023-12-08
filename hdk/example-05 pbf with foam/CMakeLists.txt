cmake_minimum_required(VERSION 3.26)

option(BuildWithEigen "Build with Eigen" OFF)
option(BuildWithCUDA "Build with CUDA" OFF)
set(TARGET PBFSolver)
set(CMAKE_CXX_STANDARD 17)

# Build With CUDA
if (BuildWithCUDA)
    project(example_04_PBF LANGUAGES CXX CUDA)
    find_package(CUDAToolkit REQUIRED)
    enable_language(CUDA)
    if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES 75)
    endif ()
else ()
    project(example_04_PBF)
endif ()


# Find Houdini
set(Houdini_PATH "C:/Program Files/Side Effects Software/Houdini 20.0.506")
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
        INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}
        pbf
)
target_compile_options(
        ${TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
if (BuildWithCUDA)
    target_compile_options(
            ${TARGET}
            INTERFACE
            $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
            $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
            $<$<COMPILE_LANGUAGE:CUDA>:-rdc=true>
    )
    target_link_libraries(${TARGET} INTERFACE ${CUDA_LIBRARIES})
    target_include_directories(${TARGET} INTERFACE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
    set_target_properties(${TARGET} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
endif ()

if (BuildWithEigen)
    find_package(Eigen3 REQUIRED)
    target_link_libraries(${TARGET} PUBLIC Eigen3::Eigen)
    add_definitions(-DUSE_EIGEN)
endif ()

houdini_configure_target(${TARGET})


if (BuildWithCUDA)
# CUDA Test
add_executable(CUDA_TEST pbf_gpu/main.cu)
target_include_directories(
        CUDA_TEST
        INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}
        pbf_gpu
)
target_compile_options(
        CUDA_TEST
        INTERFACE
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
        $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
        $<$<COMPILE_LANGUAGE:CUDA>:-rdc=true>
        --use_fast_math
)
target_link_libraries(CUDA_TEST INTERFACE ${CUDA_LIBRARIES})
target_include_directories(CUDA_TEST INTERFACE ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
set_target_properties(CUDA_TEST PROPERTIES CUDA_SEPARABLE_COMPILATION ON) # not recommended
endif ()
