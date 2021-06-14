#pragma once

#include <open3d/geometry/PointCloud.h>
#include "ListModeData.h"


namespace HUREL
{
	namespace Compton
	{
		class ReconPointCloud : open3d::geometry::PointCloud
		{
		public:
			/// ReconValue;		
			std::vector<double> reconValues_;
			void CalculateReconPoint(double(*calcFunc)(ListModeData, Eigen::Vector4d));

			double SimpleBackprojection(ListModeData lmData, Eigen::Vector4d imgPoint);
			
		};


	}
}
