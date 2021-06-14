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

		public ref class LahgiWrapper
		{
			// TODO: 여기에 이 클래스에 대한 메서드를 추가합니다.

			LahgiControl& lahgiControlInstance = LahgiControl::instance();
			RealsenseControl& realsenseControlInstance = RealsenseControl::instance();

		public:

			LahgiWrapper(eModuleManagedType type);
			bool AddListModeData(array<UINT16>^ adcData, List<array<double>^>^ echks);

			void GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE);

			void ResetListmodeData();

			void GetSLAMReconPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);

			
			void GetRealTimeReconPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs);
		};


	}
}
