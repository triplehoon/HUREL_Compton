#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <open3d/geometry/Octree.h>

#include "ListModeData.h"
#include "ReconPointCloud.h"
#include "LahgiControl.h"

namespace HUREL {
	namespace Compton
	{
		using namespace cv;
		using namespace Eigen;		
	
		class LahgiControl;
		const static bool mCodeMask[37][37] = {
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		};
		
		enum class eRadiationImagingMode
		{
			CODED,
			COMPTON,
			HYBRID
		};

		constexpr double RadiationImageFOV = 130;
		constexpr double RadiationImageSeperateDegree = 5;
		constexpr double RadiationImageResponseSurfaceDistance = 3;

		class RadiationImage
		{
		private:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:	
			
			std::vector<ListModeData> mListedListModeData;			
			Eigen::Matrix4d mDetectorTransformation;


			//		  ^
			//<-- x   | 
			//        y 
			Mat mDetectorResponseImage;

			//		  ^
			//<-- x   | 
			//        y 
			Mat mComptonImage;
			Mat mCodedImage;
			Mat mHybridImage;
		    static void ShowCV_32SAsJet(cv::Mat img, int size);
			static cv::Mat GetCV_32SAsJet(cv::Mat img, int size);
			static cv::Mat GetCV_32SAsJet(cv::Mat img, int size, double minValuePortion);

			RadiationImage(std::vector<ListModeData>& data);			
			RadiationImage(std::vector<ListModeData>& data, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov);
			double OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode);
			double OverlayValue(Eigen::Vector4d point, eRadiationImagingMode mode);

			static void OverlayRadimgToP3(cv::Mat& p3, const cv::Mat& radImg);
		};
	};
};

