#pragma once

#include <cmath>
#include <open3d/geometry/PointCloud.h>
#include "ListModeData.h"

namespace HUREL
{
	namespace Compton
	{
		struct RGBA {
			double R;
			double G;
			double B;
			double A;
		} typedef RGBA_t;
		class ReconPointCloud : public open3d::geometry::PointCloud
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			/// ReconValue;		
			ReconPointCloud(open3d::geometry::PointCloud& pc);
			std::vector<double> reconValues_;
			void CalculateReconPoint(ListModeData lmData, double(*calcFunc)(ListModeData, Eigen::Vector3d));

			static double SimpleBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint);
			static RGBA_t ColorScaleJet(double v, double vmin, double vmax);
		};



	}
}
