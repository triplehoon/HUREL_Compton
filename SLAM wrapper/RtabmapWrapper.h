#pragma once
#include "RtabmapSlamControl.h"
#include "Lahgi wrapper.h"

using namespace System;
using namespace System::IO;
using namespace System::Diagnostics;
using namespace System::Drawing;
using namespace System::Windows;
using namespace System::Threading;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace System::Runtime::InteropServices;


namespace HUREL {
	namespace Compton {

			public ref class RtabmapWrapper:IDisposable
			{
			private:
				Boolean mIsInitiated = false;
				uchar* mColorImg = nullptr;
				
			public:
				static RtabmapSlamControl& mSlamcontrolNative = RtabmapSlamControl::instance();
				
				Boolean InitiateRtabmap(System::String^% message);

				void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				void GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data);
				Boolean GetRealTimeRGBStream(int% width, int% height, int% type, array<Byte>^% data);
				Boolean StartRtabmapPipeline(System::String^% msg);
				void StopRtabmapPipeline();
				void GetReconSLAMPointCloud(double time, eReconType reconType, List<array<double>^>^% vectors, List<array<double>^>^% colors, double voxelSize);

				Boolean StartSLAM(System::String^% msg);
				void StopSLAM();

				void ResetPipeline();

				void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				void GetPoseFrame(array<double>^% mat);

				RtabmapWrapper();
				~RtabmapWrapper();
				!RtabmapWrapper();
			};

		
	}
}
