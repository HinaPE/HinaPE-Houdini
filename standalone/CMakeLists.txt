function(GEN_STANDALONE TARGET)
    add_executable(${TARGET} ${TARGET}.cpp)
    target_link_libraries(
            ${TARGET}
            PUBLIC
            Houdini
            CUDA_HinaPE
    )
    file(GLOB ALL_HDK_DLL_FILES ${_houdini_bin_dir}/*.dll)
    file(COPY ${ALL_HDK_DLL_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endfunction()

GEN_STANDALONE(__native_multithreading)
GEN_STANDALONE(__baked_kernels)
