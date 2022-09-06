#pragma once

#include <vector>

//#include "RealsenseControl.h"

#include "RtabmapSlamControl.h"

#include "Module.h"
#include "ListModeData.h"
#include "ReconPointCloud.h"
#include "Logger.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#define ACTIVE_AREA_LENGTH 0.14

namespace HUREL {
	namespace Compton {	
		class RadiationImage;
		class ReconPointCloud;

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
			std::vector<RadiationImage> mListModeImage;
			EnergySpectrum mSumSpectrum;
			EnergySpectrum mScatterSumSpectrum;
			EnergySpectrum mAbsorberSumSpectrum;
			LahgiControl();
			inline static ListModeData MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation);

			//CodeMaks Setting
			double mMaskThickness = 0.006;
		
			bool IsOnActiveArea(double x, double y, Module& module);

			bool mIsListModeGenOn = false;
			void ListModeGenPipe();

			double mListModeImgInterval;
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			static LahgiControl& instance();


			bool SetType(eMouduleType type);
			
			~LahgiControl();
			void AddListModeData(const unsigned short (byteData)[144], Eigen::Matrix4d deviceTransformation, std::vector<sEnergyCheck> eChk);
			void AddListModeDataEigen(const unsigned short (byteData)[144], Eigen::Matrix4d deviceTransformation, std::vector<sEnergyCheck> eChk);
			void AddListModeDataWithTransformation(const unsigned short byteData[], std::vector<sEnergyCheck>& eChk);
			eMouduleType GetDetectorType();

			const std::vector<ListModeData> GetListedListModeData() const;
			std::vector<ListModeData> GetListedListModeData();

			void ResetListedListModeData();
			void SaveListedListModeData(std::string filePath);			
			bool LoadListedListModeData(std::string filePath);

			EnergySpectrum GetEnergySpectrum(int fpgaChannelNumber);	
			EnergySpectrum GetSumEnergySpectrum();
			EnergySpectrum GetAbsorberSumEnergySpectrum();
			EnergySpectrum GetScatterSumEnergySpectrum();




			void ResetEnergySpectrum(int fpgaChannelNumber);

			ReconPointCloud GetReconRealtimePointCloudComptonUntransformed(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud& pc, double time);

			ReconPointCloud GetReconOverlayPointCloudCoded(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconOverlayPointCloudCompton(open3d::geometry::PointCloud& pc, double time);
			ReconPointCloud GetReconOverlayPointCloudHybrid(open3d::geometry::PointCloud& pc, double time);

			cv::Mat GetListModeImageCoded(double time);
			cv::Mat GetListModeImageCompton(double time);
			cv::Mat GetListModeImageHybrid(double time);

			void StartListModeGenPipe(double milliseconds = 500);
			void StopListModeGenPipe();
			void ResetListModeImage();

		};
	}
}

