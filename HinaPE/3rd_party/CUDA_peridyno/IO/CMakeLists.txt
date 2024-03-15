set(LIB_NAME IO)

file(
    GLOB_RECURSE LIB_SRC 
    LIST_DIRECTORIES false
    CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c*"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h*"
)

add_library(${LIB_NAME} ${LIB_SRC})
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
add_compile_definitions(CUDA_BACKEND)
set_target_properties(${LIB_NAME} PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
set_target_properties(${LIB_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)
target_link_libraries(${LIB_NAME} PUBLIC ${CUDA_LIBRARIES} Core Framework Topology)
target_link_options(${LIB_NAME} PUBLIC "/NODEFAULTLIB:library")
target_include_directories(${LIB_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)
target_compile_options(${LIB_NAME} PUBLIC -Xcompiler "/wd 4819")
target_compile_options(${LIB_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)
