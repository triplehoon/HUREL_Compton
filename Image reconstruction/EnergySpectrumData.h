#pragma once

#include <string>
#include <vector>
#include <chrono>
#include<memory>
#include "Logger.h"


namespace HUREL {
	namespace Compton {
		struct BinningEnergy {
			double Energy;
			int Count;
		};
		struct EnergyTime {
			double Energy;
			std::chrono::milliseconds InteractionTimeInMili;
		};

	};
};