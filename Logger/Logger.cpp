#include "pch.h"
#include "Logger.h"

using namespace HUREL;

void HUREL::Logger::InvokeLog(std::string className, std::string msg, eLoggerType type)
{
	for (auto var : handledFunc)
	{
		var(className, msg, type);
	}
}

void HUREL::Logger::Handle(void(*func)(std::string, std::string, eLoggerType))
{
	handledFunc.push_back(func);
}

void HUREL::Logger::Unhandle(void(*func)(std::string, std::string, eLoggerType))
{
	int i = 0;
	int findIndex = -1;
	for (auto f : handledFunc)
	{
		if (f == func)
		{
			findIndex = i;
		}
		i++;
	}
	if (findIndex == -1)
	{
		return;
	}
	else
	{
		handledFunc.erase(handledFunc.begin() + findIndex);
	}
}

Logger& HUREL::Logger::Instance()
{
	static Logger* instance = new Logger();
	return *instance;
}
