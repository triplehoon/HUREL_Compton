#pragma once
#include <string>
#include <vector>
#include <chrono>
#include<memory>
#include <concurrent_vector.h>
#include <mutex>
#include <fstream>
#include <iostream>
#include <sstream>

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
			void AddEnergy(double Energy);
			void Reset();
			EnergySpectrum operator+ (const EnergySpectrum& rhs) const;

		private:

			std::vector<BinningEnergy> mHistogramEnergy = std::vector<BinningEnergy>();
			std::vector<double> mEnergyBin = std::vector<double>();
			double mBinSize = 0;
			double mMaxEnergy = 0;
		};
	}
}


