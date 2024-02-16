function(GEN_STANDALONE TARGET)
    add_executable(${TARGET} ${TARGET}.cpp)
    target_link_libraries(
            ${TARGET}
            PUBLIC
            Houdini
            CUDA_CubbyFlow
            CUDA_HinaPE
            TreeNSearch
    )
if (MSVC)
    target_link_libraries(
            ${TARGET}
            PUBLIC
            CUDA_NeighborSearch
    )
endif ()
    file(GLOB ALL_HDK_DLL_FILES ${_houdini_bin_dir}/*.dll)
    file(COPY ${ALL_HDK_DLL_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

GEN_STANDALONE(__native_multithreading)
GEN_STANDALONE(__baked_kernels)
GEN_STANDALONE(__cunsearch)
GEN_STANDALONE(__test)
