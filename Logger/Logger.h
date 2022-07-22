#pragma once
#include "pch.h"

#include <string>
#include <vector>

namespace HUREL {	
	class Logger
	{
	private:
		std::vector<void(*)(std::string, std::string)> handledFunc = {};
		Logger(){};
	public:		
		void InvokeLog(std::string className, std::string msg);
		void Handle(void(*func)(std::string, std::string));
		void Unhandle(void(*func)(std::string, std::string));
		static Logger& Instance();
	};
};
