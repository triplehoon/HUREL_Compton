#include "RtabmapWrapper.h"

Boolean HUREL::Compton::LACC::RtabmapWrapper::InitiateRtabmap(System::String^% message)
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

void HUREL::Compton::LACC::RtabmapWrapper::GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
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

void HUREL::Compton::LACC::RtabmapWrapper::GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors)
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

void HUREL::Compton::LACC::RtabmapWrapper::GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data)
{
	delete[] mColorImg;
	if (!mSlamcontrolNative.mIsVideoStreamOn) {
		data = IntPtr::Zero;
		return;
	}
	
	mSlamcontrolNative.LockVideoFrame();
	cv::Mat color = mSlamcontrolNative.GetCurrentVideoFrame();
	
	width = color.cols;
	height = color.rows;
	stride = color.step;
	int imagesize = width * height * color.channels();
	mColorImg = new uchar[imagesize];
	memcpy(mColorImg, color.data, imagesize);
	mSlamcontrolNative.UnlockVideoFrame();



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
Boolean HUREL::Compton::LACC::RtabmapWrapper::GetRealTimeRGBStream(int% width, int% height, int% type, array<Byte>^% data)
{
	mSlamcontrolNative.LockVideoFrame();
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
		mSlamcontrolNative.UnlockVideoFrame();
		return true;
	}
	else
	{
		mSlamcontrolNative.UnlockVideoFrame();
		return false;
	}
	
	




	
}

Boolean HUREL::Compton::LACC::RtabmapWrapper::StartRtabmapPipeline(System::String^% msg)
{
	mSlamcontrolNative.StartVideoStream();
	Stopwatch^ sw = gcnew Stopwatch();
	sw->Start();
	while (!mSlamcontrolNative.mIsVideoStreamOn)
	{
		if (sw->ElapsedMilliseconds > 10000)
		{
			msg = gcnew String("Starting Slam Failed");
			return false;
		}
	}
	mSlamcontrolNative.StartSlamPipe();
	return true;
}

void HUREL::Compton::LACC::RtabmapWrapper::StopRtabmapPipeline()
{
	mSlamcontrolNative.StopSlamPipe();
	mSlamcontrolNative.StopVideoStream();
}

Boolean HUREL::Compton::LACC::RtabmapWrapper::StartSLAM(System::String^% msg)
{
	mSlamcontrolNative.StartSlamPipe();
	return true;
}

void HUREL::Compton::LACC::RtabmapWrapper::StopSLAM()
{
	mSlamcontrolNative.StopSlamPipe();
}

void HUREL::Compton::LACC::RtabmapWrapper::ResetPipeline()
{
	mSlamcontrolNative.ResetSlam();

	
}

void HUREL::Compton::LACC::RtabmapWrapper::GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
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

void HUREL::Compton::LACC::RtabmapWrapper::GetPoseFrame(array<double>^% mat)
{
	std::vector<double> transform = mSlamcontrolNative.getMatrix3DOneLineFromPoseData();
	mat = gcnew array<double>(16);
	for (int i = 0; i < 16; i++) {
		mat[i] = transform[i];
	}
}

HUREL::Compton::LACC::RtabmapWrapper::RtabmapWrapper()
{
}

HUREL::Compton::LACC::RtabmapWrapper::~RtabmapWrapper()
{
	this->!RtabmapWrapper();
}

HUREL::Compton::LACC::RtabmapWrapper::!RtabmapWrapper()
{

	delete(&mSlamcontrolNative);
}
