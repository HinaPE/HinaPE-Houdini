file(GLOB_RECURSE REACTPHYSICS3D_SOURCE *.h *.hpp *.cpp)
add_library(ReactPhysics3D STATIC ${REACTPHYSICS3D_SOURCE})
target_include_directories(
        ReactPhysics3D
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
