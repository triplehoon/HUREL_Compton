#pragma once
#include "RealsenseControl.h"
#include "LahgiControl.h"

using namespace System;
using namespace System::Diagnostics;
using namespace System::Drawing;
using namespace System::Windows;
using namespace System::Threading;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;


namespace HUREL {
	namespace Compton {
		namespace LACC {

			public ref class RealsenseControlWrapper : IDisposable
			{
			private:
				Thread^ PipelineThread;
				Thread^ SLAMThread;
				Boolean IsInitiated;
				
				Boolean^ IsSLAMOn;

				void StartRealsensePipelineNative();
				void StartSLAMNative();


				
			public:
				static  RealsenseControl* m_RealsenseControlNative = &RealsenseControl::instance();

				double AverageDepth;
				Boolean InitiateRealsense(System::String^% message);
				Boolean^ IsPipelineOn;

				RealsenseControlWrapper();

				

				void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs);
				void GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs);
				array<double>^ GetPoseFrame(int% tranckingConf);

				void GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data);

				Boolean ResetPipeline(System::String^% msg);

				Boolean StartRealsensePipeline(System::String^% msg);
				void StopRealsensePipeline();
				
				Boolean StartSLAM(System::String^% msg);
				void StopSLAM();

				void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetReconSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				bool SaveRTPointCloud(System::String^ fileName);
				bool SaveSLAMEDPointCloud(System::String^ fileName);

				// C# 소멸자. Finalize 메서드임
				~RealsenseControlWrapper();

				!RealsenseControlWrapper();

				//array<double, 2>^ GetRTPointCloud();
			};
		}
	}


}

