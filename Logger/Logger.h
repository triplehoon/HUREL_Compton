#pragma once
#include "pch.h"

#include <string>
#include <vector>

namespace HUREL {	
	enum eLoggerType
	{
		DEBUG,
		INFO,
		WARN,
		ERROR_t,
		FATAL
	};
	class Logger
	{
		
	private:
		std::vector<void(*)(std::string, std::string, eLoggerType)> handledFunc = {};
		Logger(){};
	public:		
		void InvokeLog(std::string className, std::string msg, eLoggerType type);
		void Handle(void(*func)(std::string, std::string, eLoggerType));
		void Unhandle(void(*func)(std::string, std::string, eLoggerType));
		static Logger& Instance();
	};
};
