#pragma once

#ifndef COMPTON_BP
#define COMPTON_BP
#define _USE_MATH_DEFINES 

#include <Eigen/Dense>
#include <math.h>
#include <open3d/Open3D.h>

using namespace std;
using namespace Eigen;

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
	static bool CheckEffectedBP(Vector3d scatterPhotonPosition,
		double scatterPhotonEnergy,
		Vector3d absorberPhotonPosition,
		double absorberPhotonEnergy,
		Vector3d imageSpacePosition,
		double angleThreshold);

public:
	struct ListModeData{
		Vector3d scatterPhotonPosition;
		double scatterPhotonEnergy;
		Vector3d absorberPhotonPosition;
		double absorberPhotonEnergy;
	};

	
	static double m_angleThreshold;

	static open3d::geometry::PointCloud BPtoPointCloud(vector<ListModeData>* lmdata, open3d::geometry::PointCloud* imgSpace);


};




#endif