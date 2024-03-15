set(LIB_NAME RigidBody)

file(
        GLOB_RECURSE LIB_SRC
        LIST_DIRECTORIES false
        CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*.c*"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h*"
)
add_library(${LIB_NAME} STATIC ${LIB_SRC})
target_link_libraries(${LIB_NAME} PUBLIC ${CUDA_LIBRARIES} Core IO Framework Topology)
target_include_directories(${LIB_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
add_compile_definitions(CUDA_BACKEND)
target_compile_options(${LIB_NAME} PUBLIC -Xcompiler "/wd 4819")
set_target_properties(${LIB_NAME} PROPERTIES CUDA_RESOLVE_DEVICE_SYMBOLS ON)
target_compile_options(${LIB_NAME} PUBLIC $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr;--expt-extended-lambda>)
set_target_properties(${LIB_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION OFF)
add_compile_definitions(PERIDYNO_API_EXPORTS)
