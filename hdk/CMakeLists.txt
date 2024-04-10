set(SIM_HinaPE_TARGET SIM_HinaPE)

file(GLOB_RECURSE SIM_HinaPE_HEADER *.h)
file(GLOB_RECURSE SIM_HinaPE_SOURCE *.cpp)

add_library(
        ${SIM_HinaPE_TARGET}
        SHARED
        ${SIM_HinaPE_HEADER}
        ${SIM_HinaPE_SOURCE}
)

target_link_libraries(
        ${SIM_HinaPE_TARGET}
        PUBLIC
        Houdini
        HinaPE
)
target_include_directories(
        ${SIM_HinaPE_TARGET}
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_options(
        ${SIM_HinaPE_TARGET}
        PRIVATE
        -D_LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
)
houdini_configure_target(${SIM_HinaPE_TARGET})
