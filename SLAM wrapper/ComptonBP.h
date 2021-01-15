#pragma once

#ifndef COMPTON_BP
#define COMPTON_BP
#define _USE_MATH_DEFINES 

#include <math.h>
#include <open3d/geometry/PointCloud.h>


namespace HUREL {
	class CComptonBP
	{
	private:


		/// <summary>
		/// return true if effected position is in compton cone.
		/// </summary>
		/// <param name="scatterPhotonPosition"> in mm </param>
		/// <param name="sctterPhotonEnergy"> in keV</param>
		/// <param name="absorberPhotonPosition"> in mm </param>
		/// <param name="absorberPhotonEnergy"> in keV </param>
		/// <param name="effectedPosition"> in mm </param>
		/// <param name="angleThreshold"> in (degree) ¡Æ </param>
		/// <returns></returns>
		static bool CheckEffectedBP(Eigen::Vector3d scatterPhotonPosition,
			double scatterPhotonEnergy,
			Eigen::Vector3d absorberPhotonPosition,
			double absorberPhotonEnergy,
			Eigen::Vector3d imageSpacePosition,
			double angleThreshold);

	public:
		typedef struct ListModeData {
			Eigen::Vector3d scatterPhotonPosition;
			double scatterPhotonEnergy;
			Eigen::Vector3d absorberPhotonPosition;
			double absorberPhotonEnergy;
		}ListModeData;


		double m_angleThreshold;

		static open3d::geometry::PointCloud BPtoPointCloud(std::vector<ListModeData>* lmdata, open3d::geometry::PointCloud* imgSpace, double angleThreshold);


	};
}
#endif


