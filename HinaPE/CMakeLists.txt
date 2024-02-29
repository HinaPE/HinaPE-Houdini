cmake_minimum_required(VERSION 3.26)
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
project(HinaPE LANGUAGES CXX CUDA)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(3rd_party/CUDA_CubbyFlow)
add_subdirectory(3rd_party/CUDA_NeighborSearch)
add_subdirectory(3rd_party/TreeNSearch)
option(EIGEN_BUILD_DOC OFF)
option(BUILD_TESTING OFF)
add_subdirectory(3rd_party/eigen-3.4.0)
add_subdirectory(3rd_party/Discregrid)

file(GLOB HINAPE_COMMON common/fluid.h common/neighbors.h common/kernels.h)
file(GLOB_RECURSE HINAPE_DFSPH1_SOURCE DFSPH1/*.h DFSPH1/*.cpp)
#file(GLOB_RECURSE HINAPE_DFSPH2_SOURCE DFSPH2/*.h DFSPH2/*.cpp DFSPH2/*.cu DFSPH2/*.cuh)
file(GLOB_RECURSE HINAPE_Bender2019_SOURCE Bender2019/*.h Bender2019/*.cpp)


add_library(HinaPE STATIC
        ${HINAPE_COMMON}
        ${HINAPE_DFSPH1_SOURCE}
        ${HINAPE_DFSPH2_SOURCE}
)
target_link_libraries(HinaPE PUBLIC ${CUDA_LIBRARIES} Houdini tbb CUDA_NeighborSearch TreeNSearch CUDA_CubbyFlow)
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
target_compile_options(
        HinaPE
        PUBLIC
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
        $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
        $<$<COMPILE_LANGUAGE:CUDA>:-rdc=true>
)
