#pragma once
#include <string>
#include <vector>
#include <chrono>

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
		

		class EnergySpectrum
		{
			private: 
				std::vector<BinningEnergy> mHistogramEnergy = std::vector<BinningEnergy>();
				std::vector<double> mEnergyBin = std::vector<double>();
				std::vector< EnergyTime>mEnergyList = std::vector< EnergyTime>();
			public:
				
				EnergySpectrum() {}; //do nothing
				EnergySpectrum(unsigned int binSize, double maxEnergy);
				std::vector<BinningEnergy> GetHistogramEnergy();
				std::vector<EnergyTime> GetEnergyList();
				void AddEnergy(double Energy);
				void Reset();
		};
	}
}


