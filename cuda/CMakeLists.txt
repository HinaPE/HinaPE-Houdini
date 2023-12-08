cmake_minimum_required(VERSION 3.26)
project(CUDA_Practice LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)

find_package(CUDAToolkit REQUIRED)
enable_language(CUDA)

# to remove warning
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 75)
endif()

add_library(muda INTERFACE)
target_compile_options(muda INTERFACE
        $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
        $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
        $<$<COMPILE_LANGUAGE:CUDA>:-rdc=true>
)