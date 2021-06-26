#pragma once


#define SPECTRUM_ENERGY_BIN_SIZE 5
#define SPECTRUM_MAX_ENERGY 3000

#include <string>
#include <iostream>
#include <fstream>
#include <cassert>
#include <sstream>
#include <limits>

#include <Eigen/Core>

#include "EnergySpectrum.h"


namespace HUREL {
	namespace Compton {
		enum class eMouduleType 
		{
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

				

				double mEnergyGain[10];
				double mMlpeGain[10];

				std::string mLutFileName;

				eMouduleType mModuleType;

				bool mIsModuleSet;

				bool LoadLUT(std::string FileName);
				std::tuple<unsigned int, unsigned int> FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const double(&normalizePMTValue)[9]) const;
			
			public:
				double mModuleOffsetX;
				double mModuleOffsetY;
				double mModuleOffsetZ;

				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				Module();
				Module(eMouduleType moduleType, 
						double (&eGain)[10],
						double (&mlpeGain)[10],
						std::string lutFileName,
						double moduleOffesetX = 0, double moduleOffsetX = 0, double moduleOffsetZ = 0,
						unsigned int binSize = SPECTRUM_ENERGY_BIN_SIZE, double maxEnergy = SPECTRUM_MAX_ENERGY);
				~Module();

				EnergySpectrum* _EnergySpectrum;
				const bool IsModuleSet() const;
				
				static void LoadGain(std::string fileName, eMouduleType moduleType, double* outEGain);

				const Eigen::Vector4d FastMLPosEstimation(const unsigned short pmtADCValue[]) const;

				const Eigen::Vector4d FastMLPosEstimation(unsigned short(&pmtADCValue)[36]) const;

				const double GetEcal(const unsigned short pmtADCValue[]) const;
				
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

