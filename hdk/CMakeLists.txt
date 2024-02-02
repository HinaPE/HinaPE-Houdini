add_subdirectory(Common/CUDA_CubbyFlow)
add_subdirectory(Common/CUDA_NeighborSearch)

set(HINA_TARGET SIM_HinaPE_Fluid)

file(GLOB_RECURSE HINAPE_HEADER *.h)
file(GLOB_RECURSE HINAPE_SOURCE *.cpp)

add_library(
        ${HINA_TARGET}
        SHARED
        ${HINAPE_HEADER}
        ${HINAPE_SOURCE}
)

# Link Houdini Toolkit
target_link_libraries(
        ${HINA_TARGET}
        PUBLIC
        Houdini
        CUDA_CubbyFlow
)
if (MSVC)
    target_link_libraries(
            ${HINA_TARGET}
            PUBLIC
            CUDA_NeighborSearch
    )
endif ()
target_include_directories(
        ${HINA_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(
        ${HINA_TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${HINA_TARGET})
