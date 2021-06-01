#pragma once

#define SPECTRUM_ENERGY_BIN_SIZE 5
#define SPECTRUM_MAX_ENERGY 3000

#include <string>
#include <iostream>
#include <fstream>
#include <cassert>
#include <sstream>
#include <limits>

#include <open3d/geometry/PointCloud.h>

#include "EnergySpectrum.h"


namespace HUREL {
	namespace Compton {
		enum class eMouduleType {
			MONO, // Not working.
			QUAD,
			QUAD_DUAL
		};
		class Module
		{
			private:
				double mEnergyCalibrationA = 0;
				double mEnergyCalibrationB = 1;
				double mEnergyCalibrationC = 0;

				unsigned int mLutSize = 0;
				double*** mXYLogMue = NULL;
				double** mXYSumMu = NULL;

				double mModuleOffsetX;
				double mModuleOffsetY;
				double mModuleOffsetZ;

				double mEnergyGain[9];
				double mMlpeGain[9];

				std::string mLutFileName;

				EnergySpectrum mEnergySpectrum;
				eMouduleType mModuleType;

				bool mIsModuleSet;

				bool LoadLUT(std::string FileName);
				std::tuple<unsigned int, unsigned int> FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const double(&normalizePMTValue)[9]) const;
			
			public:
				Module(eMouduleType moduleType, 
						double (&eGain)[9],
						double (&mlpeGain)[9],
						std::string lutFileName,
						double moduleOffesetX = 0, double moduleOffsetX = 0, double moduleOffsetZ = 0,
						unsigned int binSize = SPECTRUM_ENERGY_BIN_SIZE, double maxEnergy = SPECTRUM_MAX_ENERGY);
				~Module();

				const bool IsModuleSet() const;
				
				static void LoadGain(std::string fileName, eMouduleType moduleType, double* outEGain);

				const Eigen::Vector3d FastMLPosEstimation(unsigned short (&pmtADCValue)[9]) const;

				const Eigen::Vector3d  FastMLPosEstimation(unsigned short(&pmtADCValue)[36]) const;

				const double GetEcal(unsigned short(&pmtADCValue)[9]) const;
				
				/// <summary>
				/// E = a*x^2 + b*x + c
				/// </summary>
				/// <param name="a"></param>
				/// <param name="b"></param>
				/// <param name="c"></param>
				/// <returns></returns>
				void SetEnergyCalibration(double a, double b, double c);
				std::tuple<double, double, double> GetEnergyCalibration();
		};
	}
}

