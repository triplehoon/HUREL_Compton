#include "LogWrapperCaller.h"

void HUREL::Compton::WrapperCaller::Logging(std::string className, std::string msg)
{
	System::String^ classNameManage = gcnew System::String(className.c_str());
	System::String^ msgManage = gcnew System::String(msg.c_str());
	WrapperLogger::Log::Info(classNameManage, msgManage);
}
