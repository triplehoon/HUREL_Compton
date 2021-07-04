#pragma once
#include "LahgiControl.h"
#include "RealsenseControl.h"

using namespace System::Collections::Generic;
using namespace System;
using namespace System::Runtime::InteropServices;

namespace HUREL {
	namespace Compton {
		public enum class eModuleManagedType
		{
			MONO,
			QUAD,
			QUAD_DUAL
		};

		public enum class eReconType
		{
			CODED,
			COMPTON,
			HYBRID
		};

		public ref class LahgiWrapper
		{
		private:
			LahgiControl& lahgiControlInstance = LahgiControl::instance();
		/*	RealsenseControl& realsenseControlInstance = RealsenseControl::instance();*/
			size_t mReconPointsCount = 0;
			ReconPointCloud* mReconPointCloud = NULL;

		public:
			LahgiWrapper(eModuleManagedType type);
			~LahgiWrapper();
			Boolean AddListModeDataWraper(array<unsigned short>^ adcData, List<array<double>^>^ echks);

			void GetAbsoluteListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE);

			void ResetListmodeData();
			void SaveListModeData(String^ fileName);			

			void GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount);

			void GetSumSpectrum(List<array<double>^>^% energyCount);
			void GetAbsorberSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrum(List<array<double>^>^% energyCount);
			void ResetSpectrum(unsigned int channelNumber);

			void ContinueReconPointFor1m1m1m(List<array<double>^>^% vectors, List<double>^% values, double% maxValue);


		};


	}
}
