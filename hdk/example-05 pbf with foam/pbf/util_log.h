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
	MOT_Director *mot = dynamic_cast<MOT_Director *>( OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log.txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	LOGGER << V << "\n";

	LOGGER.flush();
	LOGGER.close();
}

template<typename T>
void DoLog(GA_ROHandleT<int32> Handle)
{
	MOT_Director *mot = dynamic_cast<MOT_Director *>( OPgetDirector());
	std::filesystem::path file_path(mot->getFileName().c_str());
	std::string logger_path = file_path.parent_path().string() + "/log.txt";
	std::ofstream LOGGER(logger_path, std::ios::out | std::ios::app);

	auto data = Handle->getData();
	data.size();
	LOGGER << data.size() << "\n";

	LOGGER.flush();
	LOGGER.close();
}
}

#endif //HINAPE_UTIL_LOG_H
