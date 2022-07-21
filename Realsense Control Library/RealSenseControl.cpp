#include "RealsenseControl.h"
#include <mutex>

using namespace cv;

static std::mutex mQueueRealtimePTMutex;
static std::mutex rtMutex;

RealsenseControl::RealsenseControl() :IsPipeLineOn(false), IsSLAMON(false), m_CurrentVideoFrame(Mat())
{
	
	auto stream = freopen("CONOUT$", "w", stdout);
}

bool RealsenseControl::SaveRTPointCloud(std::string& fileName)
{
	open3d::geometry::PointCloud savePointCloud = std::get<0>(this->GetRTPointCloudTransposed());
	open3d::io::WritePointCloudOption option;
	return open3d::io::WritePointCloudToPLY(fileName, savePointCloud, option);
}
bool RealsenseControl::SaveSLAMEDPointCloud(std::string& fileName)
{
	open3d::geometry::PointCloud savePointCloud = this->GetSLAMEDPointCloud();
	open3d::io::WritePointCloudOption option;
	return open3d::io::WritePointCloudToPLY(fileName, savePointCloud, option);
}

bool RealsenseControl::SaveDownSampledSLAMEDPointCloud(std::string& fileName)
{
	open3d::geometry::PointCloud savePointCloud = this->GetSLAMEDPointCloudDownSampled();
	open3d::io::WritePointCloudOption option;
	return open3d::io::WritePointCloudToPLY(fileName, savePointCloud, option);
}

RealsenseControl::~RealsenseControl()
{
	IsSLAMON = false;
	IsPipeLineOn = false;
}

RealsenseControl& RealsenseControl::instance()
{

	static RealsenseControl* instance = new RealsenseControl();
	//std::cout << "instance: " << instance << std::endl;
	return *instance;

}

bool RealsenseControl::InitiateRealsense(std::string* outMessage)
{
	try {
		ctx = rs2::context();

		cfgD455 = rs2::config();
		//cfgD455.enable_device("935322071433");	
		cfgD455.enable_stream(RS2_STREAM_COLOR, D455_H_DEPTH_SIZE, D455_V_DEPTH_SIZE, RS2_FORMAT_BGR8, 15);
		//cfgD455.enable_stream(RS2_STREAM_COLOR, D435_H_COLOR_SIZE, D435_V_COLOR_SIZE, RS2_FORMAT_RGB8, 15);
		cfgD455.enable_stream(RS2_STREAM_DEPTH, D455_H_DEPTH_SIZE, D455_V_DEPTH_SIZE, RS2_FORMAT_Z16, 15);
		pipeD455 = rs2::pipeline();

		cfgT265 = rs2::config();
		cfgT265.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		pipeT265 = rs2::pipeline(ctx);
		pipeD455.start(cfgD455);
		pipeT265.start(cfgT265);

		pipeD455.stop();
		pipeT265.stop();
	}
	catch (const rs2::camera_disconnected_error& e)
	{
		*outMessage = "Camera was disconnected! Please connect it back ";
		*outMessage = *outMessage + e.what();
		return false;
	}
	// continue with more general cases
	catch (const rs2::recoverable_error& e)
	{
		*outMessage = "Operation failed, please try again ";
		*outMessage = *outMessage + e.what();
		return false;
	}
	// you can also catch "anything else" raised from the library by catching rs2::error
	catch (const rs2::error& e)
	{
		*outMessage = "Some other error occurred! ";
		*outMessage = *outMessage + e.what();
		return false;
	}

	*outMessage = "Success";
	return true;
}
std::vector<double> RealsenseControl::getMatrix3DOneLineFromPoseData(rs2_pose poseData)
{
	Eigen::Matrix4d S_T265toLACCTransform;

	Eigen::Vector4d Quaternion = { poseData.rotation.w, -poseData.rotation.x ,poseData.rotation.y,-poseData.rotation.z };
	Eigen::Matrix3d RMat = open3d::geometry::PointCloud::GetRotationMatrixFromQuaternion(Quaternion);
	Eigen::Vector3d	TransPoseMat = { -poseData.translation.x, poseData.translation.y, -poseData.translation.z };
	Eigen::Matrix4d TransF; // Your Transformation Matrix
	TransF.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	TransF.block<3, 3>(0, 0) = RMat;
	TransF.block<3, 1>(0, 3) = TransPoseMat;
	//TransF = TransF;
	auto Matrix3Dtype = TransF.adjoint();
	std::vector<double> matrix3DOneLine;
	int idx = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			matrix3DOneLine.push_back(Matrix3Dtype(i, j));
			idx++;
		}
	}
	return matrix3DOneLine;
}
Eigen::Matrix4d RealsenseControl::GetPoseDataEigen()
{	
	rtMutex.lock();
	Eigen::Matrix4d TransF = mRTTransformationTrue;
	rtMutex.unlock();
	return TransF;
}
std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d, std::vector<Eigen::Vector2f>> RealsenseControl::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color, const rs2_pose& pose)
{
	// Object Declaration (Point Cloud)
	std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d> tupleCloudTrans;
	open3d::geometry::PointCloud cloud;
	std::vector<Eigen::Vector2f> out_uv;
	std::tuple<double, double, double> RGB_Color;
	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << -1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;


	Eigen::Vector3d D455ToT265Coord = { 0.031, -0.029, 0 };
	Eigen::Vector4d Quaternion = { pose.rotation.w, pose.rotation.x ,pose.rotation.y,pose.rotation.z };
	Eigen::Matrix3d RMat = open3d::geometry::PointCloud::GetRotationMatrixFromQuaternion(Quaternion);
	Eigen::Vector3d	TransPoseMat = { pose.translation.x, pose.translation.y, pose.translation.z };
	Eigen::Matrix4d TransF; // Your Transformation Matrix
	TransF.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	TransF.block<3, 3>(0, 0) = RMat;
	TransF.block<3, 1>(0, 3) = TransPoseMat;
	for (int i = 0; i < points.size(); i++)
	{
		if (Texture_Coord[i].u > 0 && Texture_Coord[i].v > 0 && Texture_Coord[i].u < 1 && Texture_Coord[i].v < 1)// && (Vertex[i].x) * (Vertex[i].x) + (Vertex[i].y) * (Vertex[i].y) + (Vertex[i].z) * (Vertex[i].z) < 9)
		{
			//===================================
			// Mapping Depth Coordinates
			// - Depth data stored as XYZ values
			//===================================
			/*if (-Vertex[i].y + D455ToT265Coord[1] > 0.7)
				continue;
			if (Vertex[i].z - D455ToT265Coord[2] < 0.5)
				continue;*/
			Eigen::Vector3d pointVector = { Vertex[i].x + D455ToT265Coord[0], -Vertex[i].y + D455ToT265Coord[1], -Vertex[i].z + D455ToT265Coord[2] };

			cloud.points_.push_back(pointVector);

			RGB_Color = RGB_Texture(color, Texture_Coord[i]);
			Eigen::Vector3d colorVector = { std::get<0>(RGB_Color), std::get<1>(RGB_Color),  std::get<2>(RGB_Color) }; // RGB vector
			cloud.colors_.push_back(colorVector);
			out_uv.push_back(Eigen::Vector2f(Texture_Coord[i].u, Texture_Coord[i].v));
		}
	}



	return std::make_tuple(cloud, t265toLACCAxisTransform * TransF, out_uv);


}

std::tuple<double, double, double> RealsenseControl::RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY)
{
	// Get Width and coordinates of texture
	int width = texture.get_width();  // Frame width in pixels
	int height = texture.get_height(); // Frame height in pixels
	if (Texture_XY.u < 0 || Texture_XY.v < 0)
	{
		return std::tuple<double, double, double>(0, 0, 0);
	}
	// Normals to Texture Coordinates conversion
	int x_value = std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
	//std::cerr << Text_Index;
	// RGB components to save in tuple
	double NT1 = (double)New_Texture[Text_Index] / 255;
	double NT2 = (double)New_Texture[Text_Index + 1] / 255;
	double NT3 = (double)New_Texture[Text_Index + 2] / 255;
	return std::tuple<double, double, double>(NT1, NT2, NT3);
}
void RealsenseControl::SLAMPipeline()
{
	std::shared_ptr<open3d::geometry::PointCloud > CombinedCloud_ptr(new open3d::geometry::PointCloud);
	std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud);

	double ptCloud_Voxel = 0.05;
	double Cominbed_ptCloud_Voxel = 0.05;
	double reconPtCloudDownSample = 0.05;



	int ptCount = 0;
	int maxptCout = 5;

	std::vector<open3d::geometry::PointCloud> ptClouds;
	std::vector<Eigen::Matrix4d> poses;
	auto robustRecon = new SLAMRobustRecon();
	int i = 1;

	bool isRobustReconStart = false;

	while (IsSLAMON)
	{
		if (!m_QueueRealtimeCloudTrans.empty())
		{
			mQueueRealtimePTMutex.lock();
			PC_TRANSPOS_TUPLE ptCloudPose = m_QueueRealtimeCloudTrans.front();
			m_QueueRealtimeCloudTrans.pop();
			mQueueRealtimePTMutex.unlock();
			ptClouds.push_back(std::get<0>(ptCloudPose));
			poses.push_back(std::get<1>(ptCloudPose));
			if (!isRobustReconStart) {
				robustRecon->StartRobustRecon(std::get<1>(ptCloudPose));
				isRobustReconStart = true;
			}


			/// <summary>
			/// make fragment
			/// </summary>
			if (ptClouds.size() >= maxptCout) {
				//printf("start fragment MultiwayRegisteration\n");
				//std::this_thread::sleep_for(std::chrono::milliseconds(500));

				auto ptCloud = SLAMRobustRecon::MultiwayRegisteration(ptClouds, poses);
				robustRecon->AddRobustReconPoints(std::make_tuple(ptCloud, poses[0]));
				//m_SLAMEDPointCloudDownSampled += ptCloud.Transform(T265toLACCTransform * poses[0]);
				//m_SLAMEDPointCloudDownSampled += ptCloud.Transform(poses[0]).Transform(T265toLACCTransform);
				ptClouds.clear();
				poses.clear();
				mQueueRealtimePTMutex.lock();
				if (m_QueueRealtimeCloudTrans.size() < maxptCout) {

				}
				else {
					for (int i = 0; i < m_QueueRealtimeCloudTrans.size() - maxptCout; ++i)
					{
						m_QueueRealtimeCloudTrans.pop();
					}
				}
				mQueueRealtimePTMutex.unlock();

				printf("done fragment MultiwayRegisteration\n");
			}
			else
			{
				continue;
			}
			m_SLAMEDPointCloud = robustRecon->GetSLAMEDPointCloud().Transform(T265toLACCTransform);
			m_SLAMEDPointCloudDownSampled = *open3d::geometry::PointCloud(m_SLAMEDPointCloud).VoxelDownSample(0.1);
		}
	}
	robustRecon->StopRobustRecon();

	delete(robustRecon);
}

/*void RealsenseControl::SLAMPipeline_Deprivation()
{
	std::shared_ptr<open3d::geometry::PointCloud > CombinedCloud_ptr(new open3d::geometry::PointCloud);
	std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud);

	double ptCloud_Voxel = 0.05;
	double Cominbed_ptCloud_Voxel = 0.05;
	double reconPtCloudDownSample = 0.2;

	int i = 1;
	while (IsSLAMON)
	{
		if (!m_QueueRealtimeCloudTrans.empty())
		{
			mQueueRealtimePTMutex.lock();
			auto pointPose = m_QueueRealtimeCloudTrans.front();
			m_QueueRealtimeCloudTrans.pop();
			mQueueRealtimePTMutex.unlock();

			if (i > 0) {
				auto TempPCL_T265 = std::get<0>(pointPose);//.Transform(T265toLACCTransform); //Change alignment from T265 to LACC
				Eigen::Matrix4d TransMat_init = std::get<1>(pointPose);


				if (!(*CombinedCloud_ptr).HasPoints())
				{
					*CombinedCloud_ptr = (*std::get<0>((TempPCL_T265.VoxelDownSample(ptCloud_Voxel))->RemoveStatisticalOutliers(20,5)));//.Transform(TransMat_init);
					m_SLAMEDPointCloud = *(CombinedCloud_ptr->VoxelDownSample(Cominbed_ptCloud_Voxel));
					m_SLAMEDPointCloudDownSampled = *(CombinedCloud_ptr->VoxelDownSample(reconPtCloudDownSample));
				}
				else
				{
					*pointcloud_ptr = (*std::get<0>((TempPCL_T265.VoxelDownSample(ptCloud_Voxel))->RemoveStatisticalOutliers(20, 5)));
					//Get Resgiteration pos Mat
					auto regMat = open3d::pipelines::registration::RegistrationICP(*pointcloud_ptr,*CombinedCloud_ptr,0.1);
					pointcloud_ptr->Transform(regMat.transformation_);
					*CombinedCloud_ptr = *CombinedCloud_ptr + (*pointcloud_ptr);//.Transform(TransMat_init);;
					CombinedCloud_ptr->EstimateNormals();
					m_SLAMEDPointCloud = *(CombinedCloud_ptr->VoxelDownSample(Cominbed_ptCloud_Voxel));
					m_SLAMEDPointCloudDownSampled = *(CombinedCloud_ptr->VoxelDownSample(reconPtCloudDownSample));
					/*if (m_SLAMEDPointCloudDownSampled.points_.size() > 40000) {
						reconPtCloudDownSample += 0.05;
					}
				}

			}

		}

	}
}*/
void RealsenseControl::RealsensesPipeline()
{
	m_Posedata = rs2_pose();
	rs2::pose_frame pose_frame(nullptr);

	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3);
	thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.3f);
	thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 6.0f);
	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0f);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 1.00f);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 1.0f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.3f);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 30.0f);



	pipelines.clear();

	rs2::pipeline_profile pipe_profile_D455 = pipeD455.start(cfgD455);
	float depthScale = (static_cast<rs2::depth_sensor>(pipe_profile_D455.get_device().query_sensors()[0])).get_depth_scale();
	std::cout << "depth scale: " << (static_cast<rs2::depth_sensor>(pipe_profile_D455.get_device().query_sensors()[0])).get_depth_scale() << std::endl;
	auto profiles = (static_cast<rs2::color_sensor>(pipe_profile_D455.get_device().query_sensors()[1])).get_stream_profiles();
	/*for (int i = 0; i < profiles.size(); ++i)
	{
		/*if (auto video_stream = profiles[i].as<rs2::video_stream_profile>())
		{
			try
			{
				//If the stream is indeed a video stream, we can now simply call get_intrinsics()
				rs2_intrinsics intrinsics = video_stream.get_intrinsics();

				auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
				auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
				rs2_distortion model = intrinsics.model;

				std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
				std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
				std::cout << "Distortion Model        : " << model << std::endl;
				std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
					intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
			}
			catch (const std::exception& e)
			{
				std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
			}
		}
	}
	*/
	auto depth_device = pipe_profile_D455.get_device().query_sensors()[0];
	depth_device.set_option(RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_DEFAULT);

//	depth_device.set_option(RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
	depth_device.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
	depth_device.set_option(RS2_OPTION_LASER_POWER, 360);
	rs2::pipeline_profile t265Profile = pipeT265.start(cfgT265);
	rs2::pose_sensor t265 = t265Profile.get_device().query_sensors().front();
	//t265.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 0);
	pipelines.emplace_back(pipeD455);
	pipelines.emplace_back(pipeT265);

	Sleep(1000);
	std::mutex mu;
	while (IsPipeLineOn)
	{
		for (auto&& pipe : pipelines) // loop over pipelines
		{
			rs2::frameset frames;
			if (!pipe.try_wait_for_frames(&frames))
			{
				break;
			}
			rs2::points points;

			
			auto color = frames.get_color_frame();
			auto depth = frames.get_depth_frame();
			
			auto pose = frames.get_pose_frame();


			if (color) {
			
				rs2_intrinsics intrinsics = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

				auto principal_point = std::make_pair(intrinsics.ppx, intrinsics.ppy);
				auto focal_length = std::make_pair(intrinsics.fx, intrinsics.fy);
				rs2_distortion model = intrinsics.model;

				//std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
			//std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
			//std::cout << "Distortion Model        : " << model << std::endl;
				//std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
				//intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;
					
				pc.map_to(color);
				const int w = color.as<rs2::video_frame>().get_width();
				const int h = color.as<rs2::video_frame>().get_height();
							

				m_CurrentVideoFrame = Mat(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);

				
			}
			if (depth) {
				//System::Diagnostics::Debug::WriteLine("Frame");						
				depth = dec_filter.process(depth);
				depth = thr_filter.process(depth);
				depth = spat_filter.process(depth);
				depth = temp_filter.process(depth);
				points = pc.calculate(depth);
			}
			if (pose) {
				pose_frame = pose;
			}

			if (points && pose_frame && color)
			{
				rs2_pose tempPoseData = pose_frame.get_pose_data();
				m_Posedata = tempPoseData;
				rs2::pose_frame null_pose_frame(nullptr);
				pose_frame = null_pose_frame;
				int trackerConfidence = tempPoseData.tracker_confidence;
				auto realTimeCloudPoseTransposed = PCL_Conversion(points, color, tempPoseData);


				// realTimeCloudPoseTransposed transposed from here

				rtMutex.lock();
				Eigen::Vector4d Quaternion = { m_Posedata.rotation.w, -m_Posedata.rotation.x ,m_Posedata.rotation.y,-m_Posedata.rotation.z };
				Eigen::Matrix3d RMat = open3d::geometry::PointCloud::GetRotationMatrixFromQuaternion(Quaternion);
				Eigen::Vector3d	TransPoseMat = { -m_Posedata.translation.x, m_Posedata.translation.y, -m_Posedata.translation.z };
				Eigen::Matrix4d TransF; // Your Transformation Matrix
				mRTTransformationTrue.block<3, 3>(0, 0) = RMat;
				mRTTransformationTrue.block<3, 1>(0, 3) = TransPoseMat;
				mRTTransformationTrue(3, 3) = 1;
				m_RTPointCloudTransposed = std::make_tuple(std::get<0>(realTimeCloudPoseTransposed).Transform(std::get<1>(realTimeCloudPoseTransposed)), std::get<2>(realTimeCloudPoseTransposed));
				mRTTransformation = std::get<1>(realTimeCloudPoseTransposed);

				const int w = depth.as<rs2::video_frame>().get_width();
				const int h = depth.as<rs2::video_frame>().get_height();
				Mat depth16UC = Mat(Size(w, h), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);
				Mat depth32F;
				depth16UC.convertTo(depth32F, CV_32FC1, depthScale);
				m_CurrentDepthFrame = depth32F;

				rtMutex.unlock();



				if (IsSLAMON && tempPoseData.tracker_confidence >= 2)
				{
					open3d::geometry::PointCloud untransPosedPC = open3d::geometry::PointCloud(std::get<0>(realTimeCloudPoseTransposed));
					mQueueRealtimePTMutex.lock();
					m_QueueRealtimeCloudTrans.push(std::make_tuple(untransPosedPC.Transform(std::get<1>(realTimeCloudPoseTransposed).inverse()),
						T265toLACCTransform.inverse() * std::get<1>(realTimeCloudPoseTransposed)));
					mQueueRealtimePTMutex.unlock();
				}

			}
		}
	}


	m_Posedata = rs2_pose();
	pipeD455.stop();
	pipeT265.stop();
#ifdef DEBUG
	printf("realsense lib: Finished\n");
#endif // DEBUG


	//m_CurrentVideoFrame = rs2::video_frame(nullptr);

	Sleep(1000);

}
rs2_pose RealsenseControl::GetPoseData() 
{
	return m_Posedata;
}
cv::Mat RealsenseControl::GetCurrentVideoFrame() 
{
	return m_CurrentVideoFrame;
}

cv::Mat RealsenseControl::GetCurrentDepthFrame()
{

	rtMutex.lock();

	Mat returnValue = m_CurrentDepthFrame;
	rtMutex.unlock();
	return returnValue;
}
std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> RealsenseControl::GetRTPointCloud() {
	rtMutex.lock();

	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> returnValue = m_RTPointCloud;
	rtMutex.unlock();
	return returnValue;
}

std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> RealsenseControl::GetRTPointCloudTransposed()
{
	rtMutex.lock();

	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> returnValue = m_RTPointCloudTransposed;
	rtMutex.unlock();
	return returnValue;
}


open3d::geometry::PointCloud RealsenseControl::GetSLAMEDPointCloud()
{

	return m_SLAMEDPointCloud;
}
open3d::geometry::PointCloud RealsenseControl::GetSLAMEDPointCloudDownSampled() {
	return m_SLAMEDPointCloudDownSampled;
}
