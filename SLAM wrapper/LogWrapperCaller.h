#pragma once
#include <string>
#include "Logger.h"

namespace HUREL
{
	namespace Compton
	{
		
		class WrapperCaller
		{
			
		public:
			static void Logging(std::string className, std::string msg, HUREL::eLoggerType type);
		};

	}
}