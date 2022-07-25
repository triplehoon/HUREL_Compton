#pragma once

#include <cmath>
#include <open3d/geometry/PointCloud.h>
#include <open3d/utility/Helper.h>
#include <unordered_map>

#include "ListModeData.h"
#include "RadiationImage.h"
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

		class RadiationImage;

		class ReconPointCloud : public open3d::geometry::PointCloud
		{
		private:	
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:
			
			/// ReconValue;		
			ReconPointCloud() : PointCloud() {};
			ReconPointCloud(open3d::geometry::PointCloud& pc);
			std::vector<double> reconValues_;
			double maxReoconValue = 0;

			void CalculateReconPoint(ListModeData lmData, double(*calcFunc)(ListModeData, Eigen::Vector3d));
			void CalculateReconPointCoded(RadiationImage& lmImage);
			void CalculateReconPointCompton(RadiationImage& lmImage);
			void CalculateReconPointHybrid(RadiationImage& lmImage);



			static double SimpleComptonBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint);

			static double SimpleComptonBackprojectionUntransformed(ListModeData lmData, Eigen::Vector3d imgPoint);


			static RGBA_t ColorScaleJet(double v, double vmin, double vmax);

			std::shared_ptr<ReconPointCloud> VoxelDownSample(double voxel_size) const;
		};



	}
}
