file(GLOB_RECURSE HINAPE_SOURCE *.h *.cpp *.cu *.cuh)
add_library(CUDA_HinaPE STATIC ${HINAPE_SOURCE})
target_link_libraries(CUDA_HinaPE PUBLIC ${CUDA_LIBRARIES} Houdini tbb CUDA_NeighborSearch)
target_link_directories(
        CUDA_HinaPE
        PUBLIC
        ${Houdini_PATH}/custom/houdini/dsolib
)
target_include_directories(
        CUDA_HinaPE
        PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../
)
target_compile_options(
        CUDA_HinaPE
        PUBLIC
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
        $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
        $<$<COMPILE_LANGUAGE:CUDA>:-rdc=true>
)

add_executable(HinaPE_TestDFSPH test_DFSPH.cpp)
target_link_libraries(HinaPE_TestDFSPH PUBLIC CUDA_HinaPE)
target_include_directories(HinaPE_TestDFSPH PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB ALL_HDK_DLL_FILES ${_houdini_bin_dir}/*.dll)
file(COPY ${ALL_HDK_DLL_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
