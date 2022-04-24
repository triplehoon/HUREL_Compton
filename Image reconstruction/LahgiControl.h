#pragma once

#include <vector>

#include "RealsenseControl.h"

#include "Module.h"
#include "ListModeData.h"
#include "ReconPointCloud.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#define ACTIVE_AREA_LENGTH 0.50


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
			EnergySpectrum mScatterSumSpectrum;
			EnergySpectrum mAbsorberSumSpectrum;
			LahgiControl();
			inline static ListModeData MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation);

			//CodeMaks Setting
			double mMaskThickness = 0.006;
			static double SimpleCodedBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint);
			static double mCalcMuOnCodedMask(double x, double y);
			static double mCalcIsPassOnCodedMask(double x, double y);
			bool IsOnActiveArea(double x, double y, Module& module);


		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			static LahgiControl& instance();


			void SetType(eMouduleType type);
			
			~LahgiControl();
			void AddListModeData(const unsigned short (byteData)[144], Eigen::Matrix4d deviceTransformation, std::vector<sEnergyCheck> eChk);
			void AddListModeDataWithTransformation(const unsigned short byteData[], std::vector<sEnergyCheck>& eChk);
			eMouduleType GetDetectorType();

			const std::vector<ListModeData> GetListedListModeData() const;
			std::vector<ListModeData> GetListedListModeData();

			void ResetListedListModeData();
			void SaveListedListModeData(std::string fileName);			
			void LoadListedListModeData(std::string fileName);

			EnergySpectrum GetEnergySpectrum(int fpgaChannelNumber);	
			EnergySpectrum GetSumEnergySpectrum();
			EnergySpectrum GetAbsorberSumEnergySpectrum();
			EnergySpectrum GetScatterSumEnergySpectrum();


			void ResetEnergySpectrum(int fpgaChannelNumber);

			ReconPointCloud GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconRealtimePointCloudCoded(open3d::geometry::PointCloud& pc, double time);

		};
	}
}

