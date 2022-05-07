#include "pch.h"

#include "RealsenseControlWrapper.h"

using namespace HUREL::Compton::LACC;

Boolean RealsenseControlWrapper::InitiateRealsense(System::String^% message)
{
	std::string resultMsg;

	try 
	{
		System::Diagnostics::Debug::WriteLine("Initiating Realsense");
		IsInitiated = m_RealsenseControlNative->InitiateRealsense(&resultMsg);
		message = gcnew System::String(resultMsg.c_str());
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

	open3d::geometry::PointCloud pose = std::get<0>(m_RealsenseControlNative->GetRTPointCloud());
	std::vector<Eigen::Vector2f> uv = std::get<1>(m_RealsenseControlNative->GetRTPointCloud());

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

void RealsenseControlWrapper::GetSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(IsPipelineOn || IsSLAMOn)) 
	{
		return;
	}

	open3d::geometry::PointCloud pose = m_RealsenseControlNative->GetSLAMEDPointCloud();
	
	
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


void RealsenseControlWrapper::GetReconSLAMPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();

	if (!(IsPipelineOn || IsSLAMOn))
	{
		return;
	}

	open3d::geometry::PointCloud pose = m_RealsenseControlNative->GetSLAMEDPointCloudDownSampled();

	ReconPointCloud rcPC = LahgiControl::instance().GetReconRealtimePointCloudCompton(pose, 50000);


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
		if (rcPC.reconValues_[i] > maxValue * 0.5)
		{
			array<double, 1>^ poseVector = gcnew array<double>{rcPC.points_[i][0], rcPC.points_[i][1], rcPC.points_[i][2]};
			vectors->Add(poseVector);
			RGBA_t color = ReconPointCloud::ColorScaleJet(rcPC.reconValues_[i], maxValue * 0.5, maxValue);
			array<double, 1>^ colorVector = gcnew array<double>{color.R, color.G, color.B, color.A};
			colors->Add(colorVector);
		}

	}
}

bool HUREL::Compton::LACC::RealsenseControlWrapper::SaveRTPointCloud(System::String^ fileName)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(fileName);
	std::string fileNameString = std::string(static_cast<char*>(ptrToNativeString.ToPointer()));

	return m_RealsenseControlNative->SaveRTPointCloud(fileNameString);
}

bool HUREL::Compton::LACC::RealsenseControlWrapper::SaveSLAMEDPointCloud(System::String^ fileName)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(fileName);
	std::string fileNameString = std::string(static_cast<char*>(ptrToNativeString.ToPointer()));

	return m_RealsenseControlNative->SaveSLAMEDPointCloud(fileNameString);
}

void RealsenseControlWrapper::GetRealTimeRGB(int% width, int% height, int% stride, IntPtr% data) {
	
	if (!m_RealsenseControlNative->IsPipeLineOn) {
		data = IntPtr::Zero;
		return;
	}


	/*if (m_RealsenseControlNative->m_CurrentVideoFrame == nullptr) {
		System::Diagnostics::Debug::WriteLine("m_CurrentVideoFrame null ptr");
		img = nullptr;
		return;
	}*/

	cv::Mat color = m_RealsenseControlNative->GetCurrentVideoFrame();
	


	if (color.cols > 1) {
		width = color.cols;
		height = color.rows;		
		stride = color.step;
		void* ptr = static_cast<void*>(color.ptr());

		if (ptr == NULL)
		{
			return;
		}

		data = IntPtr(ptr);
	}
}

Boolean RealsenseControlWrapper::ResetPipeline(System::String^% msg)
{
	if (!IsPipelineOn) {
		msg = "Pipeline is not on";
		return false;
	}
	this->StopRealsensePipeline();
	this->StartRealsensePipeline(msg);
	return true;
}

void RealsenseControlWrapper::StartRealsensePipelineNative()
{	
	m_RealsenseControlNative->IsPipeLineOn = true;
	m_RealsenseControlNative->RealsensesPipeline();
}

Boolean RealsenseControlWrapper::StartRealsensePipeline(System::String^% msg)
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
	m_RealsenseControlNative->IsPipeLineOn = false;
	IsPipelineOn = false;
	PipelineThread->Join();
}

array<double>^ RealsenseControlWrapper::GetPoseFrame(int% tranckingConf)
{
	tranckingConf = (int) m_RealsenseControlNative->GetPoseData().tracker_confidence;
	
	if (tranckingConf  > 0)
	{
		auto transform = m_RealsenseControlNative->getMatrix3DOneLineFromPoseData(m_RealsenseControlNative->GetPoseData());

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

Boolean RealsenseControlWrapper::StartSLAM(System::String^% msg)
{
	if (!m_RealsenseControlNative->IsPipeLineOn) {
		msg = "Realsense is not on.";
		return false;
	}	
	m_RealsenseControlNative->IsSLAMON = true;
	SLAMThread = gcnew Thread(gcnew ThreadStart(this, &RealsenseControlWrapper::StartSLAMNative));
	SLAMThread->Start();

	LahgiControl::instance().StartListModeGenPipe();
	msg = "SLAM has started";
	IsSLAMOn = true;
	return true;
}

void RealsenseControlWrapper::StartSLAMNative()
{
	m_RealsenseControlNative->SLAMPipeline();
}

void RealsenseControlWrapper::StopSLAM()
{
	LahgiControl::instance().StopListModeGenPipe();

	m_RealsenseControlNative->IsSLAMON = false;
	if (SLAMThread != nullptr)
	{
		SLAMThread->Join();
		SLAMThread = nullptr;
	}
	
	IsSLAMOn = false;	
}

void HUREL::Compton::LACC::RealsenseControlWrapper::GetReconRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	int size;
	if (!IsPipelineOn) {
		return;
	}


	auto ptcs = m_RealsenseControlNative->GetRTPointCloudTransposed();


	open3d::geometry::PointCloud points = std::get<0>(ptcs);


	ReconPointCloud rcPC = LahgiControl::instance().GetReconOverlayPointCloudHybrid(points, 50000);


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
		if (rcPC.reconValues_[i] > maxValue * 0.0)
		{
			array<double, 1>^ poseVector = gcnew array<double>{rcPC.points_[i][0], rcPC.points_[i][1], rcPC.points_[i][2]};
			vectors->Add(poseVector);
			RGBA_t color = ReconPointCloud::ColorScaleJet(rcPC.reconValues_[i], maxValue * 0.0, maxValue);
			array<double, 1>^ colorVector = gcnew array<double>{color.R, color.G, color.B, color.A};
			colors->Add(colorVector);
		}

	}
}


void RealsenseControlWrapper::GetRealTimePointCloudTransPosed(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	uvs = gcnew List< array<float>^>();
	int size;
	if (!IsPipelineOn) {
		return;
	}


	auto ptcs = m_RealsenseControlNative->GetRTPointCloudTransposed();


	open3d::geometry::PointCloud points = std::get<0>(ptcs);


	std::vector<Eigen::Vector2f> uv = std::get<1>(ptcs);




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
	vectors->Capacity = size;
	colors->Capacity = size;
	uvs->Capacity = size;

	if (uv.size() != size)
	{
		printf("Size not same size: %d uv size: %d \n", static_cast<int>(size), static_cast<int>(uv.size()));
	}


	for (int i = 0; i < size - 1; ++i) {
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

