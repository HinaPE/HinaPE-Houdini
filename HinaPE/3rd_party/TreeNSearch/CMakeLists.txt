file(GLOB_RECURSE TREENSEARCH_SOURCE *.h *.cpp)
add_library(TreeNSearch STATIC ${TREENSEARCH_SOURCE})
target_include_directories(TreeNSearch PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/extern)
target_include_directories(TreeNSearch INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
