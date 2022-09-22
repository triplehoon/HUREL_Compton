#include "RtabmapWrapper.h"

Boolean HUREL::Compton::RtabmapWrapper::InitiateRtabmap()
{
    std::string resultMsg;
    try
    {
		mIsInitiated = RtabmapCppWrapper::instance().Initiate();
	}
	catch (...)
	{
		mIsInitiated = false;
		HUREL::Compton::WrapperCaller::Logging("C++CLI::HUREL::Compton::RtabmapWrapper", "Fail to initiate", eLoggerType::ERROR_t);
	}

	if (mIsInitiated)
	{
		RtabmapCppWrapper::instance().StartVideoStream();
		return true;
	}
	else
	{
		return false;
	}

}

void HUREL::Compton::RtabmapWrapper::GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(RtabmapCppWrapper::instance().GetIsSlamPipeOn()))
	{
		return;
	}

	std::vector<ReconPointCppWrapper> pose = RtabmapCppWrapper::instance().GetRTPointCloud();


	int count = pose.size();

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorB, pose[i].colorG, pose[i].colorB};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(RtabmapCppWrapper::instance().GetIsSlamPipeOn()))
	{
		return;
	}

	std::vector<ReconPointCppWrapper> pose = RtabmapCppWrapper::instance().GetRTPointCloudTransposed();


	int count = pose.size();

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorB, pose[i].colorG, pose[i].colorB};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data)
{
	static int imagesize = 0;
	if (!RtabmapCppWrapper::instance().GetIsVideoStreamOn()) {
		width = 0;
		height = 0;
		stride = 0;
		data = IntPtr::Zero;
		return;
	}
	int inWidth = 0;
	int inHeight = 0;
	int inStride = 0;
	int inChannel = 0;
	uint8_t* inImgPtr = nullptr;
	if(RtabmapCppWrapper::instance().GetCurrentVideoFrame(&inImgPtr, &inWidth, &inHeight, &inStride, &inChannel))
	{
		data = IntPtr(inImgPtr);
		width = inWidth;
		height = inHeight;
		stride = inStride;
		return;
	}
	else
	{
		width = 0;
		height = 0;
		stride = 0;
		data = IntPtr::Zero;
		return;
	}
	
}


void HUREL::Compton::RtabmapWrapper::GetReconSLAMPointCloud(double time, eReconManaged reconType, List<array<double>^>^% vectors, List<array<double>^>^% colors, double voxelSize, bool isLoaded)
{
	eReconCppWrapper type;
	switch (reconType)
	{
	case HUREL::Compton::eReconManaged::CODED:
		type = eReconCppWrapper::CODED;
		break;
	case HUREL::Compton::eReconManaged::COMPTON:
		type = eReconCppWrapper::COMPTON;
		break;
	case HUREL::Compton::eReconManaged::HYBRID:
		type = eReconCppWrapper::HYBRID;
		break;
	default:
		break;
	}

	std::vector<ReconPointCppWrapper> pose = RtabmapCppWrapper::instance().GetReconSLAMPointCloud(time, type, voxelSize, isLoaded);
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();


	int count = pose.size();

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorR, pose[i].colorG, pose[i].colorB, pose[i].colorA};
		colors->Add(colorVector);
	}
}

Boolean HUREL::Compton::RtabmapWrapper::StartSLAM()
{
	return 	RtabmapCppWrapper::instance().StartSlamPipe();
}

void HUREL::Compton::RtabmapWrapper::StopSLAM()
{
	RtabmapCppWrapper::instance().StopSlamPipe();
}

void HUREL::Compton::RtabmapWrapper::ResetSLAM()
{
	RtabmapCppWrapper::instance().ResetSlam();
}

void HUREL::Compton::RtabmapWrapper::GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(RtabmapCppWrapper::instance().GetIsSlamPipeOn()))
	{
		return;
	}

	std::vector<ReconPointCppWrapper> pose = RtabmapCppWrapper::instance().GetSlamPointCloud();


	int count = pose.size();

	vectors->Capacity = count;
	colors->Capacity = count;

	Trace::WriteLine("SLAM points: " + count);
	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorR, pose[i].colorG, pose[i].colorB, pose[i].colorA};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetPoseFrame(array<double>^% mat)
{
	std::vector<double> transform = RtabmapCppWrapper::instance().getMatrix3DOneLineFromPoseData();
	mat = gcnew array<double>(16);
	for (int i = 0; i < 16; i++) {
		mat[i] = transform[i];
	}	//std::vector<double> transform = RtabmapCppWrapper::instance().getMatrix3DOneLineFromPoseData();
	mat = gcnew array<double>(16);
	for (int i = 0; i < 16; i++) {
		mat[i] = transform[i];
	}
}

bool HUREL::Compton::RtabmapWrapper::LoadPlyFile(System::String^ filePath)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(filePath);
	return RtabmapCppWrapper::instance().LoadPlyFile(static_cast<char*>(ptrToNativeString.ToPointer()));
}

void HUREL::Compton::RtabmapWrapper::GetLoadedPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	std::vector<ReconPointCppWrapper> pose = RtabmapCppWrapper::instance().GetLoadedPointCloud();
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	

	int count = pose.size();

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorR, pose[i].colorG, pose[i].colorB, pose[i].colorA};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetOptimizePoses(List<array<double>^>^% poses)
{
	std::vector<std::vector<double>> cppPoses = RtabmapCppWrapper::instance().GetOptimizedPoses();

	poses = gcnew List< array<double>^>();
	poses->Capacity = cppPoses.size();
	for (int i = 0; i < cppPoses.size(); ++i)
	{
		array<double, 1>^ mat = gcnew array<double>(16);
		for (int j = 0; j < 16; j++) {
			mat[j] = cppPoses[i][j];
		}
		poses->Add(mat);
	}	
}

HUREL::Compton::RtabmapWrapper::RtabmapWrapper()
{
}

HUREL::Compton::RtabmapWrapper::~RtabmapWrapper()
{
	
}

HUREL::Compton::RtabmapWrapper::!RtabmapWrapper()
{

	delete(&RtabmapCppWrapper::instance());
}
