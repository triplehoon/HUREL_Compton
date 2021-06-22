#pragma once
#include "LahgiControl.h"
#include "RealsenseControl.h"

using namespace System::Collections::Generic;
using namespace System;

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
			RealsenseControl& realsenseControlInstance = RealsenseControl::instance();


		public:

			LahgiWrapper(eModuleManagedType type);
			Boolean AddListModeDataWraper(array<unsigned short>^ adcData, List<array<double>^>^ echks);

			void GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE);

			void ResetListmodeData();

			void GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount);

			void GetSumSpectrum(List<array<double>^>^% energyCount);
			void GetAbsorberSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrum(List<array<double>^>^% energyCount);
			void ResetSpectrum(unsigned int channelNumber);

			void GetSLAMReconPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);


			void GetRealTimeReconImage(double time, List<array<double>^>^% colors, List<array<float>^>^% uvs, eReconType reconType);
		};


	}
}
