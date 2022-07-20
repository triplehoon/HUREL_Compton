#pragma once

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

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
		public:	
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
			
			RadiationImage(std::vector<ListModeData> data);			

			double OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode);
		};
	};
};

