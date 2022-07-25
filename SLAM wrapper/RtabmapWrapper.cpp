#include "RtabmapWrapper.h"

Boolean HUREL::Compton::RtabmapWrapper::InitiateRtabmap(System::String^% message)
{
    std::string resultMsg;
    try
    {
		mIsInitiated = RtabmapCppWrapper::instance().Initiate();
		message = gcnew System::String(resultMsg.c_str());
	}
	catch (...)
	{
		mIsInitiated = false;
		HUREL::Compton::WrapperCaller::Logging("C++CLI::HUREL::Compton::RtabmapWrapper", "Fail to initiate");
	}

	
    return true;
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


void HUREL::Compton::RtabmapWrapper::GetReconSLAMPointCloud(double time, eReconManaged reconType, List<array<double>^>^% vectors, List<array<double>^>^% colors, double voxelSize)
{
	//vectors = gcnew List< array<double>^>();
	//colors = gcnew List< array<double>^>();
	//int size;
	//


	//open3d::geometry::PointCloud points = *RtabmapCppWrapper::instance().GetSlamPointCloud().VoxelDownSample(voxelSize);

	//ReconPointCloud rcPC;
	//switch (reconType)
	//{
	//case HUREL::Compton::eReconType::CODED:
	//	rcPC = LahgiControl::instance()->GetReconOverlayPointCloudCoded(points, time);

	//	break;
	//case HUREL::Compton::eReconType::COMPTON:
	//	rcPC = LahgiControl::instance()->GetReconOverlayPointCloudCompton(points, time);

	//	break;
	//case HUREL::Compton::eReconType::HYBRID:
	//	rcPC = LahgiControl::instance()->GetReconOverlayPointCloudHybrid(points, time);

	//	break;
	//default:
	//	break;
	//}


	//int count = 0;
	//if (rcPC.colors_.size() < rcPC.points_.size())
	//{
	//	count = rcPC.colors_.size();
	//}
	//else
	//{
	//	count = rcPC.points_.size();
	//}

	//double maxValue = rcPC.maxReoconValue;

	//vectors->Capacity = count;
	//colors->Capacity = count;


	//for (int i = 0; i < count - 1; i++) {
	//	if (rcPC.reconValues_[i] > maxValue * 0.8)
	//	{
	//		array<double, 1>^ poseVector = gcnew array<double>{rcPC.points_[i][0], rcPC.points_[i][1], rcPC.points_[i][2]};
	//		vectors->Add(poseVector);
	//		RGBA_t color = ReconPointCloud::ColorScaleJet(rcPC.reconValues_[i], 0, maxValue);
	//		array<double, 1>^ colorVector = gcnew array<double>{color.R, color.G, color.B, color.A};
	//		colors->Add(colorVector);
	//	}

	//}
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
		array<double, 1>^ colorVector = gcnew array<double>{pose[i].colorB, pose[i].colorG, pose[i].colorR, pose[i].colorA};
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
