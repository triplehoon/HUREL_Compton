#pragma once
#include <string>
#include <vector>
#include <chrono>
#include<memory>
#include <concurrent_vector.h>
#include "Logger.h"
#include "EnergySpectrumData.h"

namespace HUREL {
	namespace Compton {
		class EnergySpectrum
		{
		public:
			EnergySpectrum() {}; //do nothing
			EnergySpectrum(unsigned int binSize, double maxEnergy);
			std::vector<BinningEnergy> GetHistogramEnergy();
			std::vector<EnergyTime> GetEnergyList();
			void AddEnergy(double Energy);
			void AddEnergy(double Energy, std::chrono::milliseconds timeInMili);
			void Reset();
			EnergySpectrum operator+(EnergySpectrum rhs);

			
		private:
			concurrency::concurrent_vector<EnergyTime> mEnergyList = concurrency::concurrent_vector< EnergyTime>();

			std::vector<BinningEnergy> mHistogramEnergy = std::vector<BinningEnergy>();
			std::vector<double> mEnergyBin = std::vector<double>();
			double mBinSize = 0;
			double mMaxEnergy = 0;
			
			
		};
	}
}


