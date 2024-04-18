cmake_minimum_required(VERSION 3.26)
cmake_policy(SET CMP0091 NEW)
set(CMAKE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
project(HinaPE LANGUAGES CXX C CUDA)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(3rd_party/CUDA_CubbyFlow)
add_subdirectory(3rd_party/CUDA_NeighborSearch)
#add_subdirectory(3rd_party/CUDA_peridyno)
add_subdirectory(3rd_party/TreeNSearch)
option(BUILD_CMD_EXECUTABLE OFF)
add_subdirectory(3rd_party/Discregrid)
add_subdirectory(3rd_party/ReactPhysics3D)

file(GLOB HINAPE_COMMON common/particles.h common/neighbors.h common/kernels.h common/geometry.h common/emitter.h)
file(GLOB_RECURSE HINAPE_DFSPH_AKINCI_SOURCE DFSPH_Akinci/*.h DFSPH_Akinci/*.cpp)
file(GLOB_RECURSE HINAPE_PBF_AKINCI_SOURCE PBF_Akinci/*.h PBF_Akinci/*.cpp)
file(GLOB_RECURSE HINAPE_PBF_SOURCE PBF/*.h PBF/*.cpp)
file(GLOB_RECURSE HINAPE_PCISPH_SOURCE PCISPH_Akinci/*.h PCISPH_Akinci/*.cpp)
#file(GLOB_RECURSE HINAPE_RIGIDBODY_SOURCE RigidBody/*.h RigidBody/*.cpp)
file(GLOB_RECURSE HINAPE_SMOKE_SOURCE Smoke/*.h Smoke/*.cpp)


add_library(HinaPE STATIC
        ${HINAPE_COMMON}
        ${HINAPE_DFSPH_AKINCI_SOURCE}
        ${HINAPE_PBF_AKINCI_SOURCE}
        ${HINAPE_PBF_SOURCE}
        ${HINAPE_PCISPH_SOURCE}
#        ${HINAPE_RIGIDBODY_SOURCE}
        ${HINAPE_SMOKE_SOURCE}
)
target_link_libraries(HinaPE PUBLIC ${CUDA_LIBRARIES} Houdini CUDA_CubbyFlow CUDA_NeighborSearch Discregrid TreeNSearch ReactPhysics3D)
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
