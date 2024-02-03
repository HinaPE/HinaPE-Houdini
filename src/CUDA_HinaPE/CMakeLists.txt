file(GLOB_RECURSE HINAPE_SOURCE *.h *.cpp *.cu)
add_library(CUDA_HinaPE STATIC ${HINAPE_SOURCE})
target_link_libraries(CUDA_HinaPE PUBLIC ${CUDA_LIBRARIES} Houdini)
target_include_directories(
        CUDA_HinaPE
        PUBLIC
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../
)
