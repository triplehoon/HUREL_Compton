#pragma once

#include <chrono>
#include <iostream>
#include <time.h>
#include <ctime>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace HUREL {
	namespace Compton {
		enum class eInterationType
		{
			NONE,
			COMPTON,
			CODED			
		};

		/// <summary>
		/// meter and keV
		/// </summary>
		struct InteractionData
		{
			Eigen::Vector4d RelativeInteractionPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
			Eigen::Vector4d TransformedInteractionPoint = Eigen::Vector4d(nan(""), nan(""), nan(""),1);
			double InteractionEnergy = 0;  
			//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		class ListModeData
		{
		public:
			//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			eInterationType Type = eInterationType::NONE;
			InteractionData Scatter;
			InteractionData Absorber;
			time_t InteractionTime = 0;
			std::chrono::milliseconds InteractionTimeInMili = std::chrono::milliseconds(0);
			Eigen::Matrix4d DetectorTransformation = Eigen::Matrix4d::Zero();

			std::string WriteListModeData();
			bool ReadListModeData(std::string data);
		};

		class PlaneReconsturctImage
		{
		private: 
			std::vector<HUREL::Compton::ListModeData> mListedListModeData;
			cv::Mat mComptonImage;
			cv::Mat mCodedImage;
			cv::Mat mHybridImage;

			time_t mInteractionInterval;
			

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			double Distance;


			PlaneReconsturctImage(double distance);
		};

	}
}



