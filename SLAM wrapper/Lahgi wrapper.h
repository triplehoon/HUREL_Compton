#pragma once
#pragma unmanaged
#include "LahgiControl.h"
#pragma managed

using namespace System;
using namespace System::Collections::Generic;
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
			HYBRID,
		};
		public ref class LahgiWrapper
		{
		private:
			LahgiControl& lahgiControlInstance = LahgiControl::instance();

		public:
			LahgiWrapper();
			bool Initiate(eModuleManagedType type);
			Boolean AddListModeDataWraper(array<unsigned short>^ adcData, List<array<double>^>^ echks);

			void GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE);

			void ResetListmodeData();

			void SaveListModeData(System::String^ fileName);

			void GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount);


			void GetSumSpectrum(List<array<double>^>^% energyCount);
			void GetAbsorberSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);
			void GetAbsorberSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);

			void ResetSpectrum(unsigned int channelNumber);

			void GetRealTimeReconImage(double time, eReconType reconType, int% width, int% height, int% stride, IntPtr% data);
			static void Logging(std::string className, std::string msg);
		};
		public class WrapperCaller
		{
		public:
			static void Logging(std::string className, std::string msg);
		};


	};
}
