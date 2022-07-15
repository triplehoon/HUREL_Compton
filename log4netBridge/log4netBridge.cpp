#include "pch.h"
#include <atlstr.h>


#include "log4netBridge.h"

//#include <atlstr.h>

using namespace System;

/// <summary>

/// Example of how to simply configure and use log4net

/// </summary>
ref class LoggingExample
{
private:
    // Create a logger for use in this class
    static log4net::ILog^ log;
    static LoggingExample()
    {

    }

public:
	static void ReportMessageWarning(const char* msg)
	{
        try
        {
            log = log4net::LogManager::GetLogger("");
            String^ data = gcnew String(msg);
            log->Warn(data);
        }
        catch(Exception^ ex)
        {
            Console::WriteLine(ex->ToString());
            int a = 1;
        }        
	}
	static void ReportMessageError(const char* msg)
	{
        log = log4net::LogManager::GetLogger("");
		String^ data = gcnew String(msg);
		log->Error(data);
	}

	static void ReportMessageInfo(const char* msg)
	{
        log = log4net::LogManager::GetLogger("");
		String^ data = gcnew String(msg);
		log->Info(data);
	}static void ReportMessageDebug(const char* msg)
	{
        log = log4net::LogManager::GetLogger("");
		String^ data = gcnew String(msg);
		log->Debug(data);
      }
};

extern "C"
{

    _declspec(dllexport) void ReportMessageWarning(const char* msg)
    {
        LoggingExample::ReportMessageWarning(msg);
    }
    _declspec(dllexport) void ReportMessageError(const char* msg)
    {
        LoggingExample::ReportMessageError(msg);
    }

    _declspec(dllexport) void ReportMessageInfo(const char* msg)
    {
        LoggingExample::ReportMessageInfo(msg);
    }

    _declspec(dllexport) void ReportMessageDebug(const char* msg)
    {
        LoggingExample::ReportMessageDebug(msg);
    }
}