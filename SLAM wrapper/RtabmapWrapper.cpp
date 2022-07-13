#include "RtabmapWrapper.h"

Boolean HUREL::Compton::RtabmapWrapper::InitiateRtabmap(System::String^% message)
{
    std::string resultMsg;
    //try
    {
		System::Diagnostics::Debug::WriteLine("Initiating Rtabmap");
		mIsInitiated = mSlamcontrolNative.Initiate(&resultMsg);
		message = gcnew System::String(resultMsg.c_str());
		System::Diagnostics::Debug::WriteLine("PipeLine Setting Compelete");
	}
	//catch (...)
	{
		//mIsInitiated = false;
		//System::Diagnostics::Trace::WriteLine("Initiating Rtabmap Failed. Unkown Error");
		//message = "Initiating Rtabmap Failed. Unkown Error";
	}

	
    return mIsInitiated;
}

void HUREL::Compton::RtabmapWrapper::GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(mSlamcontrolNative.mIsSlamPipeOn))
	{
		return;
	}

	open3d::geometry::PointCloud pose = mSlamcontrolNative.GetRTPointCloud();


	int count = 0;
	if (pose.colors_.size() < pose.points_.size())
	{
		count = pose.colors_.size();
	}
	else
	{
		count = pose.points_.size();
	}

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(mSlamcontrolNative.mIsSlamPipeOn))
	{
		return;
	}

	open3d::geometry::PointCloud pose = mSlamcontrolNative.GetRTPointCloudTransposed();


	int count = 0;
	if (pose.colors_.size() < pose.points_.size())
	{
		count = pose.colors_.size();
	}
	else
	{
		count = pose.points_.size();
	}

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data)
{
	static int imagesize = 0;
	if (!mSlamcontrolNative.mIsVideoStreamOn) {
		width = 0;
		height = 0;
		stride = 0;
		data = IntPtr::Zero;
		return;
	}
	
	//SlamcontrolNative.LockVideoFrame();
	cv::Mat color = mSlamcontrolNative.GetCurrentVideoFrame();
	if (color.cols == 0)
	{
		//mSlamcontrolNative.UnlockVideoFrame();
		return;
	}
	width = color.cols;
	height = color.rows;
	stride = color.step;
	if (imagesize != width * height * color.channels())
	{
		Console::WriteLine("size diff");
		imagesize = width * height * color.channels();
		delete[] mColorImg;
		mColorImg = new uchar[imagesize];
	}
	
	memcpy(mColorImg, color.data, imagesize);
	//mSlamcontrolNative.UnlockVideoFrame();



	if (color.cols > 1) {
		width = color.cols;
		height = color.rows;
		stride = color.step;
		void* ptr = static_cast<void*>(mColorImg);

		if (ptr == NULL)
		{
			return;
		}

		data = IntPtr(ptr);
	}
}
Boolean HUREL::Compton::RtabmapWrapper::GetRealTimeRGBStream(int% width, int% height, int% type, array<Byte>^% data)
{
	//mSlamcontrolNative.LockVideoFrame();
	cv::Mat color = mSlamcontrolNative.GetCurrentVideoFrame();
	if (color.cols > 1) 
	{
		width = color.cols;
		height = color.rows;
		type = color.type();
		int imagesize = width * height * color.channels();
		data = gcnew array<Byte>(imagesize);
		for (int i = 0; i < imagesize; ++i)
		{
			data[i] = color.data[i];
		}		
		//mSlamcontrolNative.UnlockVideoFrame();
		return true;
	}
	else
	{
		//mSlamcontrolNative.UnlockVideoFrame();
		return false;
	}
	
	




	
}

Boolean HUREL::Compton::RtabmapWrapper::StartRtabmapPipeline(System::String^% msg)
{
	mSlamcontrolNative.StartVideoStream();
	Stopwatch^ sw = gcnew Stopwatch();
	sw->Start();
	while (!mSlamcontrolNative.mIsVideoStreamOn)
	{
		if (sw->ElapsedMilliseconds > 10000)
		{
			return false;
		}
	}
	mSlamcontrolNative.StartSlamPipe();
	return true;
}

void HUREL::Compton::RtabmapWrapper::StopRtabmapPipeline()
{
	mSlamcontrolNative.StopSlamPipe();
	mSlamcontrolNative.StopVideoStream();
}

void HUREL::Compton::RtabmapWrapper::GetReconSLAMPointCloud(double time, eReconType reconType, List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	int size;
	


	open3d::geometry::PointCloud points = *mSlamcontrolNative.GetSlamPointCloud().VoxelDownSample(0.1);

	ReconPointCloud rcPC;
	switch (reconType)
	{
	case HUREL::Compton::eReconType::CODED:
		rcPC = LahgiControl::instance().GetReconOverlayPointCloudCoded(points, time);

		break;
	case HUREL::Compton::eReconType::COMPTON:
		rcPC = LahgiControl::instance().GetReconOverlayPointCloudCompton(points, time);

		break;
	case HUREL::Compton::eReconType::HYBRID:
		rcPC = LahgiControl::instance().GetReconOverlayPointCloudHybrid(points, time);

		break;
	default:
		break;
	}


	int count = 0;
	if (rcPC.colors_.size() < rcPC.points_.size())
	{
		count = rcPC.colors_.size();
	}
	else
	{
		count = rcPC.points_.size();
	}

	double maxValue = rcPC.maxReoconValue;

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		if (rcPC.reconValues_[i] > maxValue * 0.8)
		{
			array<double, 1>^ poseVector = gcnew array<double>{rcPC.points_[i][0], rcPC.points_[i][1], rcPC.points_[i][2]};
			vectors->Add(poseVector);
			RGBA_t color = ReconPointCloud::ColorScaleJet(rcPC.reconValues_[i], 0, maxValue);
			array<double, 1>^ colorVector = gcnew array<double>{color.R, color.G, color.B, color.A};
			colors->Add(colorVector);
		}

	}
}

Boolean HUREL::Compton::RtabmapWrapper::StartSLAM(System::String^% msg)
{
	mSlamcontrolNative.StartSlamPipe();
	return true;
}

void HUREL::Compton::RtabmapWrapper::StopSLAM()
{
	mSlamcontrolNative.StopSlamPipe();
}

void HUREL::Compton::RtabmapWrapper::ResetPipeline()
{
	LahgiControl::instance().StopListModeGenPipe();

	mSlamcontrolNative.ResetSlam();
	LahgiControl::instance().StartListModeGenPipe();
	
}

void HUREL::Compton::RtabmapWrapper::GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(mSlamcontrolNative.mIsSlamPipeOn))
	{
		//Console::WriteLine("ttest1\n");
		return;
	}
	//Console::WriteLine("ttest2\n");
	open3d::geometry::PointCloud pose = mSlamcontrolNative.GetSlamPointCloud();


	int count = 0;
	if (pose.colors_.size() < pose.points_.size())
	{
		count = pose.colors_.size();
	}
	else
	{
		count = pose.points_.size();
	}

	vectors->Capacity = count;
	colors->Capacity = count;


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);
		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void HUREL::Compton::RtabmapWrapper::GetPoseFrame(array<double>^% mat)
{
	std::vector<double> transform = mSlamcontrolNative.getMatrix3DOneLineFromPoseData();
	mat = gcnew array<double>(16);
	for (int i = 0; i < 16; i++) {
		mat[i] = transform[i];
	}
}

HUREL::Compton::RtabmapWrapper::RtabmapWrapper()
{
}

HUREL::Compton::RtabmapWrapper::~RtabmapWrapper()
{
	this->!RtabmapWrapper();
	delete[] mColorImg;
}

HUREL::Compton::RtabmapWrapper::!RtabmapWrapper()
{

	delete(&mSlamcontrolNative);
}
