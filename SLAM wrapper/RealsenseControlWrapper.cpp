#include "pch.h"

#include "RealsenseControlWrapper.h"

using namespace HUREL::Compton::LACC;

Boolean RealsenseControlWrapper::InitiateRealsense(String^% message)
{
	std::string resultMsg;

	

	try 
	{
		System::Diagnostics::Debug::WriteLine("Initiating Realsense");
		IsInitiated = m_RealsenseControlNative->InitiateRealsense(&resultMsg);
		message = gcnew String(resultMsg.c_str());
		System::Diagnostics::Debug::WriteLine("PipeLine Setting Compelete");
	}
	catch(...)
	{
		IsInitiated = false;
		System::Diagnostics::Trace::WriteLine("Initiating Realsense Failed. Unkown Error");
		message = "Initiating Realsense Failed. Unkown Error";
	}

	return IsInitiated;
}

HUREL::Compton::LACC::RealsenseControlWrapper::RealsenseControlWrapper()
{
}

void RealsenseControlWrapper::GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, double xOffset, double yOffset, double zOffset)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!IsPipelineOn) {
		return;
	}

	open3d::geometry::PointCloud pose = m_RealsenseControlNative->m_RTPointCloud;
	if (pose.colors_.size() != pose.points_.size())
		return;

	for (int i = 0; i < pose.colors_.size(); i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0] - xOffset, pose.points_[i][1] - yOffset, pose.points_[i][2] - zOffset};
		vectors->Add(poseVector);

		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void RealsenseControlWrapper::GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, double xOffset, double yOffset, double zOffset)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	//Width 640 Heigh 480

	if (!(IsPipelineOn || IsSLAMOn)) 
	{
		return;
	}

	open3d::geometry::PointCloud pose = m_RealsenseControlNative->m_SLAMEDPointCloud;
	int count = 0;
	if (pose.colors_.size() < pose.points_.size())
	{
		count = pose.colors_.size();
	}
	else 
	{
		count = pose.points_.size();
	}


	for (int i = 0; i < count - 1; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0] - xOffset, pose.points_[i][1] - yOffset, pose.points_[i][2] - zOffset};
		vectors->Add(poseVector);

		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void RealsenseControlWrapper::GetRealTimeRGB(Bitmap^% img) {
	
	if (img == nullptr) {
		throw gcnew ArgumentNullException();
	}

	if (!m_RealsenseControlNative->m_IsPipeLineOn) {
		img = nullptr;
		return;
	}


	/*if (m_RealsenseControlNative->m_CurrentVideoFrame == nullptr) {
		System::Diagnostics::Debug::WriteLine("m_CurrentVideoFrame null ptr");
		img = nullptr;
		return;
	}*/

	rs2::video_frame* color = &m_RealsenseControlNative->m_CurrentVideoFrame;
	


	if (*color) {

		if (color->get_data() == nullptr) {
			System::Diagnostics::Debug::WriteLine("color.getdata null ptr");
			img == nullptr;
			return;
		}

		int width = color->get_width();
		int height = color->get_height();
		int stride = color->get_stride_in_bytes();

		void* ptr = (void*)color->get_data();
	

		IntPtr data = IntPtr(ptr);

		img = gcnew Bitmap(width, height, stride, System::Drawing::Imaging::PixelFormat::Format24bppRgb, data);
	}
}

Boolean RealsenseControlWrapper::ResetPipeline(String^% msg)
{
	if (!IsPipelineOn) {
		msg = "Pipeline is not on";
		return false;
	}
	this->StopRealsensePipeline();
	this->StartRealsensePipeline(msg);
	return true;
	// TODO: insert return statement here
}

void RealsenseControlWrapper::StartRealsensePipelineNative()
{	
	m_RealsenseControlNative->m_IsPipeLineOn = true;
	m_RealsenseControlNative->RealsensesPipeline();
}

Boolean RealsenseControlWrapper::StartRealsensePipeline(String^% msg)
{
	if (!IsInitiated) {
		msg = "Realsense need to initiate";
		return false;
	}
	PipelineThread = gcnew Thread(gcnew ThreadStart(this,&RealsenseControlWrapper::StartRealsensePipelineNative));
	PipelineThread->Start();
	msg = "Realsense pipeline starts";
	IsPipelineOn = true;
	return true;
}

void RealsenseControlWrapper::StopRealsensePipeline()
{
	m_RealsenseControlNative->m_IsPipeLineOn = false;
	IsPipelineOn = false;
	PipelineThread->Join();
}

array<double> ^ RealsenseControlWrapper::GetPoseFrame(int% tranckingConf)
{
	tranckingConf = (int) m_RealsenseControlNative->m_Posedata.tracker_confidence;
	
	if (tranckingConf >= 3)
	{
		auto transform = m_RealsenseControlNative->getMatrix3DOneLineFromPoseData(m_RealsenseControlNative->m_Posedata);

		array<double>^ tempArray = gcnew array<double>(16);
		for (int i = 0; i < 16; i++) {
			tempArray[i] = transform[i];
		}
		return tempArray;
	}
	else
	{
		return nullptr;
	}
}

Boolean RealsenseControlWrapper::StartSLAM(String^% msg)
{
	if (!m_RealsenseControlNative->m_IsPipeLineOn) {
		msg = "Realsense is not on.";
		return false;
	}
	m_RealsenseControlNative->m_SLAMEDPointCloud = open3d::geometry::PointCloud();
	m_RealsenseControlNative->m_IsSLAMON = true;
	SLAMThread = gcnew Thread(gcnew ThreadStart(this, &RealsenseControlWrapper::StartSLAMNative));
	SLAMThread->Start();
	msg = "SLAM has started";
	IsSLAMOn = true;
}

void RealsenseControlWrapper::StartSLAMNative()
{
	m_RealsenseControlNative->SLAMPipeline();
}

void RealsenseControlWrapper::StopSLAM()
{
	m_RealsenseControlNative->m_IsSLAMON = false;
	SLAMThread->Join();
	IsSLAMOn = false;	
}

RealsenseControlWrapper::~RealsenseControlWrapper()
{
	this->!RealsenseControlWrapper();
}

RealsenseControlWrapper::!RealsenseControlWrapper()
{
	if (PipelineThread != nullptr) {
		StopRealsensePipeline();
	}

	if (SLAMThread != nullptr) {
		StopSLAM();
	}

	delete(m_RealsenseControlNative);
}

