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

				unsigned int mBinSize;
				double mMaxEnergy;

			public:
				EnergySpectrum():mMaxEnergy(0),mBinSize(0) {}; //do nothing
				EnergySpectrum(unsigned int binSize, double maxEnergy);
				std::vector<BinningEnergy> GetHistogramEnergy();
				
				std::vector<double> CalcPeakEnergys();
				
				void AddEnergy(double Energy);
				void Reset();
		};
	}
}


