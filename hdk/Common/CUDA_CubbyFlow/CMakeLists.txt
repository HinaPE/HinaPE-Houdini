find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)
if (NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75)
endif ()
set(CUDA_LINK_LIBRARIES_KEYWORD PUBLIC)
set(CUDA_SEPARABLE_COMPILATION OFF)
set(CUDA_STANDARD 17)

file(GLOB_RECURSE CUDA_CUBBYFLOW_SOURCE *.h *.hpp *.cpp *.cu)
add_library(CUDA_CubbyFlow STATIC ${CUDA_CUBBYFLOW_SOURCE})
target_link_libraries(CUDA_CubbyFlow PUBLIC ${CUDA_LIBRARIES})
target_include_directories(
        CUDA_CubbyFlow
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/tinyobj
        ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers
        ${CMAKE_CURRENT_SOURCE_DIR}/Core
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${DEFAULT_LINKER_OPTIONS}
        ${DEFAULT_LIBRARIES}
)
target_compile_definitions(
        CUDA_CubbyFlow
        PUBLIC
        CUBBYFLOW_USE_CUDA
        ${DEFAULT_COMPILE_OPTIONS}
)
