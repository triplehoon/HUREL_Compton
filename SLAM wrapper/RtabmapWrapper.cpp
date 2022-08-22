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

	//
	////SlamcontrolNative.LockVideoFrame();
	//cv::Mat color = RtabmapCppWrapper::instance().GetCurrentVideoFrame();
	//if (color.cols == 0)
	//{
	//	//RtabmapCppWrapper::instance().UnlockVideoFrame();
	//	return;
	//}
	//width = color.cols;
	//height = color.rows;
	//stride = color.step;
	//if (imagesize != width * height * color.channels())
	//{
	//	Console::WriteLine("size diff");
	//	imagesize = width * height * color.channels();
	//	delete[] mColorImg;
	//	mColorImg = new uchar[imagesize];
	//}
	//
	//memcpy(mColorImg, color.data, imagesize);
	////RtabmapCppWrapper::instance().UnlockVideoFrame();



	//if (color.cols > 1) {
	//	width = color.cols;
	//	height = color.rows;
	//	stride = color.step;
	//	void* ptr = static_cast<void*>(mColorImg);

	//	if (ptr == NULL)
	//	{
	//		return;
	//	}

	//	data = IntPtr(ptr);
	//}
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


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose[i].pointX, pose[i].pointY, pose[i].pointZ};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorB, pose[i].colorG, pose[i].colorB};
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
