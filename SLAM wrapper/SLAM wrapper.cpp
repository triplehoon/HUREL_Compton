#include "pch.h"

#include "SLAM wrapper.h"



void SLAMwrapper::Realsense_Control::InitiateRealsense()
{
	
	//rs2::pointcloud pc;
	//rs2::points points;
	//rs2::pose_frame pose_frame(nullptr);
	//rs2::context                          ctx;        // Create librealsense context for managing devices
	//std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
	//std::vector<rs2::pipeline>            pipelines;
	//rs2::pipeline pipeD435(ctx);
	//rs2::config cfgD435;
	System::Diagnostics::Debug::WriteLine("Configure Pipelines");
	realsenseVariables->ctx = rs2::context();
	// D435 Pipeline Setting
	//realsenseVariables->cfgD435 = rs2::config();
	System::Diagnostics::Debug::WriteLine("Configure D435");
	realsenseVariables->cfgD435 = rs2::config();
	realsenseVariables->cfgD435.enable_device("935322071433");
	realsenseVariables->pipeD435 = rs2::pipeline();
	Sleep(1000);
	realsenseVariables->pipeD435.start(realsenseVariables->cfgD435);
	realsenseVariables->pipelines.emplace_back(realsenseVariables->pipeD435);

	//T265 Pipeline Setting
	System::Diagnostics::Debug::WriteLine("Configure T265");
	realsenseVariables->cfgT265 = rs2::config();
	realsenseVariables->cfgT265.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
	realsenseVariables->pipeT265 = rs2::pipeline(realsenseVariables->ctx);
	Sleep(1000);
	realsenseVariables->pipeT265.start(realsenseVariables->cfgT265);
	realsenseVariables->pipelines.emplace_back(realsenseVariables->pipeT265);

	System::Diagnostics::Debug::WriteLine("PipeLine Setting Compelete");
	realsenseVariables->m_IsPipeLineOn = true;
	Thread ^ pipeThread = gcnew Thread(gcnew ThreadStart(SLAMwrapper::Realsense_Control::StartPipeLine));
	pipeThread->Start();
}


void SLAMwrapper::Realsense_Control::GetRealTimePointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors , System::Boolean ^ isMLPEOn)
{
	vectors = gcnew List< array<double>^>();
	colors = gcnew List< array<double>^>();
	//Width 640 Heigh 480
	
	open3d::geometry::PointCloud pose = realsenseVariables->m_RTPointCloud;
	if (pose.colors_.size() != pose.points_.size())
		return;

	if (isMLPEOn) {
		HUREL::CComptonBP::BPtoPointCloud(m_LMDataVec, &pose, 5);
	}

	for (int i = 0; i < pose.colors_.size(); i++) {		
		array<double, 1>^ poseVector = gcnew array<double>{pose.points_[i][0], pose.points_[i][1], pose.points_[i][2]};
		vectors->Add(poseVector);
		
		array<double, 1>^ colorVector = gcnew array<double>{pose.colors_[i][0], pose.colors_[i][1], pose.colors_[i][2]};
		colors->Add(colorVector);
	}
	


}


void SLAMwrapper::Realsense_Control::StartPipeLine() {
	realsenseVariables->StartPipeLine();
}



array<double> ^ SLAMwrapper::Realsense_Control::GetPoseFrame()
{
	if (realsenseVariables->posedata.tracker_confidence>2)
	{
		auto transform = realsenseVariables->getMatrix3DOneLineFromPoseData(realsenseVariables->posedata);

		float^ x = realsenseVariables->posedata.translation.x;
		float^ y = realsenseVariables->posedata.translation.y;
		float^ z = realsenseVariables->posedata.translation.z;
		double dx = System::Convert::ToDouble(x);
		double dy = System::Convert::ToDouble(y);
		double dz = System::Convert::ToDouble(z);
		Console::WriteLine("x: {0} , y: {1} , z: {2}", dx* 100.0, dy*100, dz*100);
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

void SLAMwrapper::Realsense_Control::SetLMData(List<array<double>^>^ scatterPhotonPosition, List<double>^ scatterPhotonEnergy, 
											   List<array<double>^>^ absoberPhotonPosition, List<double>^ absorberPhotonEnergy)
{
	auto lmDataSet = new std::vector<HUREL::CComptonBP::ListModeData>();
	for (int i = 0; i < scatterPhotonEnergy->Count; i++) {
		auto lmData = new HUREL::CComptonBP::ListModeData();
		auto scPose = scatterPhotonPosition[i];
		auto scEnergy = scatterPhotonEnergy[i];
		auto abPose = absoberPhotonPosition[i];
		auto abEnergy = absorberPhotonEnergy[i];
		lmData->absorberPhotonPosition = Eigen::Vector3d();
		lmData->absorberPhotonPosition[0] = abPose[0];
		lmData->absorberPhotonPosition[1] = abPose[1];
		lmData->absorberPhotonPosition[2] = abPose[2];

		lmData->absorberPhotonEnergy = abEnergy;

		lmData->scatterPhotonPosition = Eigen::Vector3d();
		lmData->scatterPhotonPosition[0] = scPose[0];
		lmData->scatterPhotonPosition[1] = scPose[1];
		lmData->scatterPhotonPosition[2] = scPose[2];
		lmData->scatterPhotonEnergy = scEnergy;		
		lmDataSet->push_back(*lmData);
	}

	m_LMDataVec = lmDataSet;
}

