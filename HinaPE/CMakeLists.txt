cmake_minimum_required(VERSION 3.26)
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
project(HinaPE LANGUAGES CXX C CUDA)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(3rd_party/CUDA_CubbyFlow)
add_subdirectory(3rd_party/CUDA_NeighborSearch)
add_subdirectory(3rd_party/TreeNSearch)
#option(EIGEN_BUILD_DOC OFF)
#option(BUILD_TESTING OFF)
#add_subdirectory(3rd_party/eigen-3.4.0)
option(BUILD_CMD_EXECUTABLE OFF)
add_subdirectory(3rd_party/Discregrid)
add_subdirectory(3rd_party/ReactPhysics3D)

file(GLOB HINAPE_COMMON common/particles.h common/neighbors.h common/kernels.h common/geometry.h common/emitter.h)
file(GLOB_RECURSE HINAPE_DFSPH_AKINCI_SOURCE
        DFSPH_Akinci/*.h DFSPH_Akinci/*.cpp
        DFSPH_Akinci_Dynamics/*.h DFSPH_Akinci_Dynamics/*.cpp
)


add_library(HinaPE STATIC
        ${HINAPE_COMMON}
        ${HINAPE_DFSPH_AKINCI_SOURCE}
)
target_link_libraries(HinaPE PUBLIC ${CUDA_LIBRARIES} Houdini tbb CUDA_CubbyFlow CUDA_NeighborSearch Discregrid TreeNSearch ReactPhysics3D)
target_link_directories(
        HinaPE
        PUBLIC
        ${Houdini_PATH}/custom/houdini/dsolib
)
target_include_directories(
        HinaPE
        PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../
)

set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
add_library(HinaPE_CUDA
        CUDA/DFSPH_CUDA.cu
        CUDA/DFSPH_CUDA.h

        CUDA/include/CudaAllocator.h
        CUDA/include/CudaTexture.h
        CUDA/include/helper_cuda.h
        CUDA/include/helper_string.h
        CUDA/include/ticktock.h
        CUDA/include/wangsrng.h
)
target_link_libraries(HinaPE_CUDA PUBLIC ${CUDA_LIBRARIES} CUDA_NeighborSearch)
target_include_directories(HinaPE_CUDA
        PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_SOURCE_DIR}
        CUDA/include
        3rd_party/eigen-3.4.0
)
target_compile_options(HinaPE_CUDA PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)
target_compile_options(HinaPE_CUDA PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)

add_executable(
        test CUDA/test.cu
)
target_link_libraries(test PUBLIC HinaPE_CUDA)
