#pragma once





#include "RealsenseControl.h"
#include "ComptonBP.h"

using namespace Intel::RealSense;
using namespace System;
using namespace System::Drawing;
using namespace System::Windows;
using namespace System::Threading;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace HUREL::Compton;

namespace SLAMwrapper {

	public ref class Realsense_Control
	{

		static  RealsenseControl* realsenseVariables = new RealsenseControl();
		static void StartPipeLine();
		static std::vector<HUREL::CComptonBP::ListModeData>* m_LMDataVec = new std::vector<HUREL::CComptonBP::ListModeData>();

	public:
		void InitiateRealsense();
		void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, System::Boolean^ isMLPEOn);
		array<double>^ GetPoseFrame();
		void SetLMData(List<array<double>^>^ scatterPhotonPosition, List<double>^ scatterPhotonEnergy,
					List<array<double>^>^ absoberPhotonPosition, List<double>^ absorberPhotonEnergy);
		/*void StartSLAMPointClouds();
		void StopSLAMPOintClouds();
*/
		
		

		//array<double, 2>^ GetRTPointCloud();
	};


}

