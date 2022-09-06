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
#include "Logger.h"

namespace HUREL {
	namespace Compton {
		enum class eMouduleType 
		{			
			MONO, // Not working.
			QUAD,
			QUAD_DUAL,
			TEST
		};		

		class Module
		{
		private:
			
			double mEnergyCalibrationA = 0;
			double mEnergyCalibrationB = 1;
			double mEnergyCalibrationC = 0;

			unsigned int mLutSize = 0;
			double*** mXYLogMue = nullptr;
			double** mXYSumMu = nullptr;

			Eigen::Array<double, 1, 9>** mXYLogMueEigen = nullptr;

			double mGain[10];
			Eigen::Array<double, 1, 9> mGainEigen;

			std::string mModuleName = "";
			eMouduleType mModuleType = HUREL::Compton::eMouduleType::MONO;
			EnergySpectrum mEnergySpectrum = EnergySpectrum(5, 3000);
			bool mIsModuleSet = false;

			bool LoadGain(std::string fileName, eMouduleType moduleType, double* outEGain);
			bool LoadLUT(std::string FileName);
			std::tuple<unsigned int, unsigned int> FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const double(&normalizePMTValue)[9]) const;
			std::tuple<unsigned int, unsigned int> FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const Eigen::Array<double, 1, 9>& pmtADCValue) const;
			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
				double mModuleOffsetX;
				double mModuleOffsetY;
				double mModuleOffsetZ;

				
				Module();
				Module(eMouduleType moduleType, 
						std::string configDir, std::string moduleName,
						double moduleOffesetX = 0, double moduleOffsetY = 0, double moduleOffsetZ = 0,
						unsigned int binSize = SPECTRUM_ENERGY_BIN_SIZE, double maxEnergy = SPECTRUM_MAX_ENERGY);
				~Module();

				
				const bool IsModuleSet() const;
								
				const Eigen::Vector4d FastMLPosEstimation(const unsigned short pmtADCValue[]) const;
				const Eigen::Vector4d FastMLPosEstimation(const Eigen::Array<double, 1, 9>& pmtADCValue) const;
				const Eigen::Vector4d FastMLPosEstimation(unsigned short(&pmtADCValue)[36]) const;

				const double GetEcal(const unsigned short pmtADCValue[]) const;
				const double GetEcal(const Eigen::Array<double, 1, 9>& pmtADCValue) const;


				EnergySpectrum& GetEnergySpectrum();
				const std::string GetModuleName() const;
				bool SetGain(eMouduleType type, std::vector<double> gain);

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

