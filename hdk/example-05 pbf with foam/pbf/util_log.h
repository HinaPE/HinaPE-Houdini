#ifndef HINAPE_UTIL_LOG_H
#define HINAPE_UTIL_LOG_H

#include <OP/OP_Director.h>
#include <MOT/MOT_Director.h>
#include <filesystem>
#include <string>

namespace HinaPE
{
template<typename T, int Size = 1>
void DoLog(T log, const std::string &file_name)
{
	MOT_Director *mot = dynamic_cast<MOT_Director *>(OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log_" + file_name + ".txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	if constexpr (Size == 1)
		LOGGER << log << "\n";
	else
	{
		for (int i = 0; i < Size; ++i)
			LOGGER << log[i] << " ";
		LOGGER << "\n";
	}

	LOGGER.flush();
	LOGGER.close();
}

template<typename T, int Size = 1>
void DoLog(std::vector<T> vec, const std::string &file_name)
{
#ifdef HINA_DEBUG
	MOT_Director *mot = dynamic_cast<MOT_Director *>(OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log_" + file_name + ".txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	LOGGER << "Size:" << vec.size() << ": ";
	if constexpr (Size == 1)
	{
		for (auto &v: vec)
			LOGGER << v << " ";
		LOGGER << "\n";
	} else
	{
		for (auto &v: vec)
		{
			for (int i = 0; i < Size; ++i)
				LOGGER << v[i] << " ";
			LOGGER << "/ ";
		}
		LOGGER << "\n";
	}

	LOGGER.flush();
	LOGGER.close();
#endif
}

template<typename T>
void DoLog(GA_ROHandleT<int32> Handle, const std::string &file_name)
{
#ifdef HINA_DEBUG
	MOT_Director *mot = dynamic_cast<MOT_Director *>( OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log_" + file_name + ".txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	auto data = Handle->getData();
	data.size();
	LOGGER << data.size() << "\n";

	LOGGER.flush();
	LOGGER.close();
#endif
}

template<typename T, int Size = 1>
void InfoLog(T Log, const std::string &Prefix = "")
{
#ifdef HINA_DEBUG
	std::string total("Info: ");
	total += Prefix;
	if constexpr (Size == 1)
		total += Log;
	else
	{
		for (int i = 0; i < Size; ++i)
			total += Log[i];
	}
	DoLog<std::string, Size>(total, "info");
#endif
}

template<typename T, int Size = 1>
void MathLog(T Log, const std::string &Prefix = "")
{
#ifdef HINA_DEBUG
	std::string total("Math: ");
	total += Prefix;
	total += Log;
	DoLog<std::string, Size>(total, "math");
#endif
}

template<typename T, int Size = 1>
void ErrorLog(T Log, const std::string &Prefix = "")
{
#ifdef HINA_DEBUG
	std::string total("Error: ");
	total += Prefix;
	total += Log;
	DoLog<std::string, Size>(total, "error");
#endif
}
} // namespace HinaPE

//std::vector<int> vec1{1, 2, 3};
//HinaPE::DoLog(vec1, "info");
//std::vector<Eigen::Vector3f> vec3{Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(), Eigen::Vector3f::Identity()};
//HinaPE::DoLog<Eigen::Vector3f, 3>(vec3, "info");

#endif //HINAPE_UTIL_LOG_H
