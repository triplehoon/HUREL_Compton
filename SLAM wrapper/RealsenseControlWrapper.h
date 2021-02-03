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
				Boolean InitiateRealsense(String^% message);

				RealsenseControlWrapper();


				void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, double xOffset, double yOffset, double zOffset);
				array<double>^ GetPoseFrame(int% tranckingConf);

				void GetRealTimeRGB(Bitmap^% img);

				Boolean ResetPipeline(String^% msg);

				Boolean StartRealsensePipeline(String^% msg);
				void StopRealsensePipeline();
				
				Boolean StartSLAM(String^% msg);
				void StopSLAM();

				void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, double xOffset, double yOffset, double zOffset);

				// C# 소멸자. Finalize 메서드임
				~RealsenseControlWrapper();

				!RealsenseControlWrapper();

				//array<double, 2>^ GetRTPointCloud();
			};
		}
	}


}

