#pragma once
#pragma managed(push, off)
#include "CppWrapper.h"
#pragma managed(pop)

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

		public ref struct sBitmapWrapper
		{
			IntPtr ptr;
			int width;
			int height;
			int stride;
			sBitmapWrapper(IntPtr _ptr, int _width, int _height, int _stride)
			{
				ptr = _ptr;
				width = _width;
				height = _height;
				stride = _stride;
			}
			sBitmapWrapper(sBitMapUnmanged data)
			{
				ptr = IntPtr(data.ptr);
				width = data.width;
				height = data.height;
				stride = data.step;
			}
		};

		public ref class LahgiWrapper
		{
		private:
		public:			
			LahgiWrapper();
			bool Initiate(eModuleManagedType type);
			void AddListModeDataWraper(array<unsigned short>^ adcData);

			void SetEchks(List<array<double>^>^ echks);

			void GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE);
			Int64 GetListedListModeDataSize();

			void ResetListmodeData();

			void SaveListModeData(System::String^ fileName);

			bool LoadListModeData(System::String^ filePath);

			void GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount);

			void GetEcal(unsigned int channelNumer, double% ecalA, double% ecalB, double% ecalC);
			void SetEcal(unsigned int channelNumer, double ecalA, double ecalB, double ecalC);



			void GetSumSpectrum(List<array<double>^>^% energyCount);
			void GetAbsorberSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrum(List<array<double>^>^% energyCount);
			void GetScatterSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);
			void GetAbsorberSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time);



			void ResetSpectrum(unsigned int channelNumber);


			sBitmapWrapper^ GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter);
				

			Tuple< sBitmapWrapper^, sBitmapWrapper^, sBitmapWrapper^>^ Get2dRadationImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion);

			sBitmapWrapper^ GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution);

			void GetRealTimeReconImage(double time, eReconType reconType, int% width, int% height, int% stride, IntPtr% data);
			static void Logging(std::string className, std::string msg);
		};



	};
}
