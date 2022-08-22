#include "LogWrapperCaller.h"


void HUREL::Compton::WrapperCaller::Logging(std::string className, std::string msg, HUREL::eLoggerType type)
{
	System::String^ classNameManage = gcnew System::String(className.c_str());
	System::String^ msgManage = gcnew System::String(msg.c_str());

	switch (type)
	{
	case DEBUG:
		WrapperLogger::Log::Debug(classNameManage, msgManage);
		break;
	case INFO:
		WrapperLogger::Log::Info(classNameManage, msgManage);
		break;
	case WARN:
		WrapperLogger::Log::Warn(classNameManage, msgManage);
		break;
	case ERROR_t:
		WrapperLogger::Log::Error(classNameManage, msgManage);
		break;
	case FATAL:
		WrapperLogger::Log::Fatal(classNameManage, msgManage);
		break;
	default:
		break;
	}	
}
