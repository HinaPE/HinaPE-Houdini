file(GLOB_RECURSE CUDA_CUBBYFLOW_SOURCE *.h *.hpp *.cpp *.cu)
if (APPLE)
    list(REMOVE_ITEM CUDA_CUBBYFLOW_SOURCE Core/CUDA/*)
endif ()
add_library(CUDA_CubbyFlow STATIC ${CUDA_CUBBYFLOW_SOURCE})
target_link_libraries(CUDA_CubbyFlow PUBLIC ${CUDA_LIBRARIES} tbb)
if (MSVC)
    target_link_directories(
            CUDA_CubbyFlow
            PUBLIC
            ${Houdini_PATH}/custom/houdini/dsolib
    )
endif ()
target_include_directories(
        CUDA_CubbyFlow
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/tinyobj
        ${CMAKE_CURRENT_SOURCE_DIR}/flatbuffers
        ${CMAKE_CURRENT_SOURCE_DIR}/Core
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
        ${_houdini_include_dir}
)
target_compile_definitions(
        CUDA_CubbyFlow
        PUBLIC
        CUBBYFLOW_USE_CUDA
        CUBBYFLOW_TASKING_TBB
)
