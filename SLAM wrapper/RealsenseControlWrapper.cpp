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

void RealsenseControlWrapper::GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	int size;
	if (!IsPipelineOn) {
		return;
	}
	


	open3d::geometry::PointCloud pose = std::get<0>(m_RealsenseControlNative->m_RTPointCloud);
	std::vector<Eigen::Vector2f> uv = std::get<1>(m_RealsenseControlNative->m_RTPointCloud);


	if (pose.IsEmpty()) {
		return;		
	}

	if (pose.colors_.size() > pose.points_.size())
	{
		size = pose.points_.size();
	}
	else 
	{
		size = pose.colors_.size();
	}
	if (uv.size() < size) {
		size = uv.size();
	}

	if (uv.size() == 0)
	{
		return;
	}
	double averageDepth = 0.0;
	for (int i = 0; i < size; i++) {
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);
		averageDepth = averageDepth + System::Double( pose.points_[i][2]);
		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	
		array<float, 1>^ uvVector = gcnew array<float>{uv[i][0], uv[i][1]};
		uvs->Add(uvVector);

	}
	AverageDepth = averageDepth / size;
}

void RealsenseControlWrapper::GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	int size;
	if (!IsPipelineOn) {
		return;
	}



	open3d::geometry::PointCloud points = std::get<0>(m_RealsenseControlNative->m_RTPointCloudTransposed);
	std::vector<Eigen::Vector2f> uv = std::get<1>(m_RealsenseControlNative->m_RTPointCloudTransposed);


	if (points.IsEmpty()) {
		return;
	}

	if (points.colors_.size() > points.points_.size())
	{
		size = points.points_.size();
	}
	else
	{
		size = points.colors_.size();
	}
	if (uv.size() < size) {
		size = uv.size();
	}

	if (uv.size() == 0)
	{
		return;
	}
	double averageDepth = 0.0;
	for (int i = 0; i < size; ++i) {
		array<double, 1>^ poseVector = gcnew array<double>{points.points_[i][0], points.points_[i][1], points.points_[i][2]};
		vectors->Add(poseVector);
		averageDepth = averageDepth + System::Double(points.points_[i][2]);
		array<double, 1>^ colorVector = gcnew array<double>{points.colors_[i][2], points.colors_[i][1], points.colors_[i][0]};
		colors->Add(colorVector);

		array<float, 1>^ uvVector = gcnew array<float>{uv[i][0], uv[i][1]};
		uvs->Add(uvVector);

	}
	AverageDepth = averageDepth / size;
}



void RealsenseControlWrapper::GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

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
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);

		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void RealsenseControlWrapper::GetReconSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(IsPipelineOn || IsSLAMOn))
	{
		return;
	}

	open3d::geometry::PointCloud pose = m_RealsenseControlNative->m_SLAMEDPointCloudDownSampled;
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
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);

		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][2], pose.colors_[i][1], pose.colors_[i][0]};
		colors->Add(colorVector);
	}
}

void RealsenseControlWrapper::GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data) {
	
	if (!m_RealsenseControlNative->m_IsPipeLineOn) {
		data = IntPtr::Zero;
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
			data == IntPtr::Zero;
			return;
		}

		width = color->get_width();
		height = color->get_height();
		stride = color->get_stride_in_bytes();

		void* ptr = (void*)color->get_data();


		data = IntPtr(ptr);

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
	
	if (tranckingConf  > 0)
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
	if (SLAMThread != nullptr)
	{
		SLAMThread->Join();
		SLAMThread = nullptr;
	}
	
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

