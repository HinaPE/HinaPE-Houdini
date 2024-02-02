add_executable(_native_multithreading native_multithreading.cpp)
target_link_libraries(
        _native_multithreading
        PUBLIC
        Houdini
)
file(GLOB ALL_HDK_DLL_FILES ${_houdini_bin_dir}/*.dll)
file(COPY ${ALL_HDK_DLL_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
