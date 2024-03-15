set(LIB_NAME Core)

file(GLOB_RECURSE PERIDYNO_CORE_SOURCE *.h *.hpp *.cpp *.cu)
add_library(${LIB_NAME} STATIC ${PERIDYNO_CORE_SOURCE})
target_link_libraries(${LIB_NAME} PUBLIC ${CUDA_LIBRARIES})
target_include_directories(${LIB_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        Backend/Cuda
        ../../eigen-3.4.0
        ../glm-0.9.9.7)

add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
add_compile_definitions(CUDA_BACKEND)
target_compile_options(${LIB_NAME} PUBLIC -Xcompiler "/wd 4819")
target_link_options(${LIB_NAME} PUBLIC "/NODEFAULTLIB:library")
set_target_properties(${LIB_NAME} PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
#To disable the warning "calling a constexpr __host__ function("***") from a __host__ __device__ function("***") is not allowed."
target_compile_options(${LIB_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
