#pragma once
#include "RealsenseControl.h"


using namespace System;
using namespace System::Diagnostics;
using namespace System::Drawing;
using namespace System::Windows;
using namespace System::Threading;
using namespace System::Collections;
using namespace System::Collections::Generic;


namespace HUREL {
	namespace Compton {
		namespace LACC {

			public ref class RealsenseControlWrapper : IDisposable
			{
			private:
				Thread^ PipelineThread;
				Thread^ SLAMThread;
				Boolean IsInitiated;
				Boolean^ IsPipelineOn;
				Boolean^ IsSLAMOn;

				void StartRealsensePipelineNative();
				void StartSLAMNative();

				static  RealsenseControl* m_RealsenseControlNative = new RealsenseControl();
				
			public:
				double AverageDepth;
				Boolean InitiateRealsense(String^% message);

				RealsenseControlWrapper();

				

				void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs);
				void GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs);
				array<double>^ GetPoseFrame(int% tranckingConf);

				void GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data);

				Boolean ResetPipeline(String^% msg);

				Boolean StartRealsensePipeline(String^% msg);
				void StopRealsensePipeline();
				
				Boolean StartSLAM(String^% msg);
				void StopSLAM();

				void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void RealsenseControlWrapper::GetReconSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				// C# 소멸자. Finalize 메서드임
				~RealsenseControlWrapper();

				!RealsenseControlWrapper();

				//array<double, 2>^ GetRTPointCloud();
			};
		}
	}


}

