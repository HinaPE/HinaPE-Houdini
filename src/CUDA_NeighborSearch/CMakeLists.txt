if (MSVC)
file(GLOB_RECURSE CUDA_NEIGHBOR_SEARCH_SOURCE *.h *.cpp *.cu)
add_library(CUDA_NeighborSearch STATIC ${CUDA_NEIGHBOR_SEARCH_SOURCE})
include(GenerateExportHeader)
generate_export_header(CUDA_NeighborSearch
        BASE_NAME cuNSearch
        EXPORT_MACRO_NAME cuNSearch_EXPORT
        EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/cuNSearch/cuNSearch_export.h
)
target_link_libraries(CUDA_NeighborSearch PUBLIC ${CUDA_LIBRARIES})
target_include_directories(
        CUDA_NeighborSearch
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Utils
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../
)
file(COPY ${CMAKE_CURRENT_BINARY_DIR}/cuNSearch/cuNSearch_export.h DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include)

#target_compile_options(
#        CUDA_NeighborSearch
#        PUBLIC
#        -DCUNSEARCH_USE_DOUBLE_PRECISION
#)
endif ()