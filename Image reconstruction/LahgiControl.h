#pragma once

#include <vector>
#include "Module.h"
#include "ListModeData.h"

namespace HUREL {
	namespace Compton {	
		struct sEnergyCheck
		{
			double minE;
			double maxE;
		};
		
		class LahgiControl
		{
			private:		
			Module** mScatterModules;  //./Module information/MONOScatter1/Gain.csv, LUT.csv ...
			Module** mAbsorberModules;	//./Module information/QUADScatter1/Gain.csv, LUT.csv ...
			eMouduleType mModuleType;
			std::vector<ListModeData> mListedListModeData;
			
			EnergySpectrum mSumSpectrum;
			LahgiControl();
			inline static ListModeData MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double scatterEnergy, double absorberEnergy, Eigen::Matrix4d& transformation);

			public:
			static LahgiControl& instance();


			void SetType(eMouduleType type);
			
			~LahgiControl();
			void AddListModeData(const unsigned short (&byteData)[144], Eigen::Matrix4d& deviceTransformation, std::vector<sEnergyCheck>& eChk);
			eMouduleType GetDetectorType();

			const std::vector<ListModeData> GetListedListModeData() const;
			void ResetListedListModeData();
			void SaveListedListModeData(std::string fileName);

			EnergySpectrum GetEnergySpectrum(int fpgaChannelNumber);			

		};
	}
}

