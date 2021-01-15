#pragma once





#include "RealsenseControl.h"
#include "ComptonBP.h"

using namespace System;
using namespace System::Drawing;
using namespace System::Windows;
using namespace System::Threading;
using namespace System::Collections;
using namespace System::Collections::Generic;

namespace SLAMwrapper {

	public ref class Realsense_Control : IDisposable
	{

		static  RealsenseControl* realsenseVariables = new RealsenseControl();
		static void StartPipeLine();
		static void StartSLAMThread();
		static std::vector<HUREL::CComptonBP::ListModeData>* m_LMDataVec = new std::vector<HUREL::CComptonBP::ListModeData>();

	public:
		void InitiateRealsense();
		void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, System::Boolean^ isMLPEOn);
		array<double>^ GetPoseFrame();
		void SetLMData(List<array<double>^>^ scatterPhotonPosition, List<double>^ scatterPhotonEnergy,
					List<array<double>^>^ absoberPhotonPosition, List<double>^ absorberPhotonEnergy);

		void StartSLAM();
		void StopSLAM();
		void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);

		// C# 소멸자. Finalize 메서드임
		~Realsense_Control();

		//array<double, 2>^ GetRTPointCloud();
	};


}

