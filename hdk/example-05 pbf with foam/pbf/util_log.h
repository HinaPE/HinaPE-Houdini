#ifndef HINAPE_UTIL_LOG_H
#define HINAPE_UTIL_LOG_H

#include <OP/OP_Director.h>
#include <MOT/MOT_Director.h>
#include <filesystem>

namespace HinaPE
{
template<typename T>
void ErrorLog(T V)
{
#ifdef HINA_DEBUG
	MOT_Director *mot = dynamic_cast<MOT_Director *>(OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log.txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	LOGGER << V << "\n";

	LOGGER.flush();
	LOGGER.close();
#endif
}

template<typename T>
void DoLog(std::vector<T> vec)
{
#ifdef HINA_DEBUG
	MOT_Director *mot = dynamic_cast<MOT_Director *>(OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log.txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	LOGGER << "Size:" << vec.size() << ": ";
	for (auto &v : vec)
		LOGGER << v << " ";
	LOGGER << "\n";

	LOGGER.flush();
	LOGGER.close();
#endif
}

template<typename T>
void DoLog(std::vector<T> vec, int size)
{
#ifdef HINA_DEBUG
	MOT_Director *mot = dynamic_cast<MOT_Director *>(OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log.txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	LOGGER << "Size:" << vec.size() << ": ";
	for (auto &v : vec)
		for (int i = 0; i < size; ++i)
			LOGGER << v[i] << " ";
	LOGGER << "\n";

	LOGGER.flush();
	LOGGER.close();
#endif
}

template<typename T>
void DoLog(GA_ROHandleT<int32> Handle)
{
#ifdef HINA_DEBUG
	MOT_Director *mot = dynamic_cast<MOT_Director *>( OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log.txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	auto data = Handle->getData();
	data.size();
	LOGGER << data.size() << "\n";

	LOGGER.flush();
	LOGGER.close();
#endif
}
} // namespace HinaPE

#endif //HINAPE_UTIL_LOG_H
