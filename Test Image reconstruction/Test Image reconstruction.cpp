// Test Image reconstruction.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
#include <chrono>

#include "TestFuncs.h"

#define ARRAYSIZE (100)

using namespace std;
using namespace chrono;
using namespace HUREL;
using namespace Compton;

void ConsoleWrite(string className, string msg, eLoggerType type);

int main()
{
	
	Logger::Instance().Handle(ConsoleWrite);
	LahgiControl& control = HUREL::Compton::LahgiControl::instance();
	control.SetType(eMouduleType::QUAD);
	TestFuncs::TestAddingLmDataVerification(5000);
	//TestFuncs::TestAddingLmData();
	
}


void ConsoleWrite(string className, string msg, eLoggerType type)
{
	string typeString;
	switch (type)
	{
	case HUREL::DEBUG:
		typeString = "DEBUG";
		break;
	case HUREL::INFO:
		typeString = "INFO";
		break;
	case HUREL::WARN:
		typeString = "WARN";
		break;
	case HUREL::ERROR_t:
		typeString = "ERROR";
		break;
	case HUREL::FATAL:
		typeString = "FATAL";
		break;
	default:
		break;
	}

	cout << "[" << typeString << "]" << className << ": " << msg << endl;
	
}