#pragma once
#pragma managed(push, off)
#include "CppWrapper.h"
#pragma managed(pop)

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

		public enum class eReconManaged
		{			
			CODED,
			COMPTON,
			HYBRID
		};
			public ref class RtabmapWrapper:IDisposable
			{
			private:
				Boolean mIsInitiated = false;				
			public:				
				Boolean InitiateRtabmap();

				void GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				void GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data);

				void GetReconSLAMPointCloud(double time, eReconManaged reconType, List<array<double>^>^% vectors, List<array<double>^>^% colors, double voxelSize, bool useLoaded);
				
				Boolean StartSLAM();
				void StopSLAM();
				void ResetSLAM();

				void StopVideoStream();

				void GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);

				void GetPoseFrame(array<double>^% mat);

				bool LoadPlyFile(System::String^ filePath);
				void GetLoadedPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors);
				void GetOptimizePoses(List<array<double>^>^% poses);

				void SavePlyFile(System::String^ filePath);


				RtabmapWrapper();
				~RtabmapWrapper();
				!RtabmapWrapper();
			};

		
	}
}
