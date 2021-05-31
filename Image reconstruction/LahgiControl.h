#pragma once

#include <vector>

#include "Module.h"
#include "ListModeData.h"

namespace HUREL {
	namespace Compton {	
		class LahgiControl
		{
			private:
			Module mScatterModules[8];  //./Module information/MONOScatter1/Gain.csv, LUT.csv ...
			Module mAbsorberModules[8];	//./Module information/QUADScatter1/Gain.csv, LUT.csv ...
			eMouduleType mModuleType;
			std::vector<ListModeData> mListedListModeData;
			
			EnergySpectrum mSumSpectrum;

			public:
			LahgiControl(eMouduleType type);
			~LahgiControl();
			void AddListModeData(unsigned short (&byteData)[144]);
			eMouduleType GetDetectorType();

			const std::vector<ListModeData> GetListedListModeData() const;
			void ResetListedListModeData();
			void SaveListedListModeData(std::string fileName);

			EnergySpectrum GetEnergySpectrum(int fpgaChannelNumber);
		};
	}
}

