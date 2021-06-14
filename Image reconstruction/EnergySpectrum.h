#pragma once

#include <vector>

namespace HUREL {
	namespace Compton {
		struct BinningEnergy {
			double Energy;
			int Count;
		};
		

		class EnergySpectrum
		{
			private: 
				std::vector<BinningEnergy> mHistogramEnergy;
				std::vector<double> mEnergyBin;
			public:
				EnergySpectrum() {}; //do nothing
				EnergySpectrum(unsigned int binSize, double maxEnergy);
				std::vector<BinningEnergy> GetHistogramEnergy();
				void AddEnergy(double Energy);
				void Reset();
		};
	}
}


