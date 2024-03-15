#pragma once
#define PERIDYNO_VERSION 0.9.1
#define PERIDYNO_VERSION_MAJOR 0
#define PERIDYNO_VERSION_MINOR 9
#define PERIDYNO_VERSION_PATCH 1


#if ((defined _WIN32) || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(_DLL)
#if !defined(PERIDYNO_DLL) && !defined(PERIDYNO_STATIC)
#define PERIDYNO_DLL
#endif
#endif

#if ((defined _WIN32) || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(PERIDYNO_DLL)
#define PERIDYNO_EXPORT extern "C" __declspec(dllexport)
#define PERIDYNO_IMPORT extern "C" __declspec(dllimport)
#else
#define PERIDYNO_EXPORT
#define PERIDYNO_IMPORT
#endif

#if defined(PERIDYNO_API_EXPORTS)
#define PERIDYNO_API PERIDYNO_EXPORT
#else
#define PERIDYNO_API PERIDYNO_IMPORT
#endif

#ifndef CUDA_BACKEND
#define CUDA_BACKEND
#endif

#if(defined(CUDA_BACKEND))
#include <cuda_runtime.h>
#	define DYN_FUNC __device__ __host__ 
#	define GPU_FUNC __device__ 
#	define CPU_FUNC __host__ 
#else
#	define DYN_FUNC
#	define GPU_FUNC 
#	define CPU_FUNC 
#endif

enum DeviceType
{
	CPU,
	GPU,
	UNDEFINED
};

#define PRECISION_FLOAT

#include "Typedef.inl"
const inline std::string getAssetPath() {
#if defined(VK_USE_PLATFORM_ANDROID_KHR)
	return "";
#elif defined(VK_EXAMPLE_DATA_DIR)
	return VK_EXAMPLE_DATA_DIR;
#else
	return "C:/Users/xayah/Desktop/HinaPE-peridyno/data/";
#endif
}

const inline std::string getPluginPath() {
	return "C:/Users/xayah/Desktop/HinaPE-peridyno/cmake-build-release/plugin/";
}

//#define SIMULATION2D
