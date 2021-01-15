
#include "RealsenseControl.h"



RealsenseControl::RealsenseControl()
{
	m_IsSLAMON = false;
	CurrentVideoFrame = new rs2::video_frame(nullptr);
}

RealsenseControl::~RealsenseControl()
{
	m_IsSLAMON = false;
	m_IsPipeLineOn = false;
	delete(CurrentVideoFrame);
}

std::vector<double> RealsenseControl::getMatrix3DOneLineFromPoseData(rs2_pose poseData)
{

	Eigen::Vector4d Quaternion = { poseData.rotation.w, poseData.rotation.x ,poseData.rotation.y,poseData.rotation.z };
	Eigen::Matrix3d RMat = open3d::geometry::PointCloud::GetRotationMatrixFromQuaternion(Quaternion);
	Eigen::Matrix4d T265toLACCTransform;
	T265toLACCTransform << -1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;
	Eigen::Matrix4d TransF; // Your Transformation Matrix
	TransF.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	TransF.block<3, 3>(0, 0) = RMat;
	TransF = TransF * T265toLACCTransform;
	auto Matrix3Dtype = TransF.adjoint();
	std::vector<double> matrix3DOneLine;
	int idx = 0;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			matrix3DOneLine.push_back(Matrix3Dtype(i, j));
			idx++;
		}
	}
	return matrix3DOneLine;
}

std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d> RealsenseControl::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color, const rs2_pose& pose)
{
	// Object Declaration (Point Cloud)
	std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d> tupleCloudTrans;
	open3d::geometry::PointCloud cloud;
	std::tuple<double, double, double> RGB_Color;
	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();
	Eigen::Matrix4d T265toLACCTransform;
	T265toLACCTransform << -1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;


	Eigen::Vector3d D436ToT265Coord = { -0.017, 0.03, 0 };
	Eigen::Vector3d	TransPoseMat = { -pose.translation.x, pose.translation.y, -pose.translation.z };
	Eigen::Vector4d Quaternion = { pose.rotation.w, pose.rotation.x ,pose.rotation.y,pose.rotation.z };
	Eigen::Matrix3d RMat = open3d::geometry::PointCloud::GetRotationMatrixFromQuaternion(Quaternion);
	Eigen::Matrix4d TransF; // Your Transformation Matrix
	TransF.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	TransF.block<3, 3>(0, 0) = RMat;
	TransF.block<3, 1>(0, 3) = TransPoseMat;
	for (int i = 0; i < points.size(); i++)
	{
		if (Texture_Coord[i].u > 0 && Texture_Coord[i].v > 0 && Texture_Coord[i].u <1 && Texture_Coord[i].v <1)// && (Vertex[i].x) * (Vertex[i].x) + (Vertex[i].y) * (Vertex[i].y) + (Vertex[i].z) * (Vertex[i].z) < 9)
		{
			//===================================
			// Mapping Depth Coordinates
			// - Depth data stored as XYZ values
			//===================================
			if (-Vertex[i].y + D436ToT265Coord[1] > 0.7)
				continue;
			Eigen::Vector3d pointVector = { Vertex[i].x + D436ToT265Coord[0], -Vertex[i].y + D436ToT265Coord[1], -Vertex[i].z + D436ToT265Coord[2] };
			cloud.points_.push_back(pointVector);

			RGB_Color = RGB_Texture(color, Texture_Coord[i]);
			Eigen::Vector3d colorVector = { std::get<0>(RGB_Color), std::get<1>(RGB_Color),  std::get<2>(RGB_Color) }; // RGB vector
			cloud.colors_.push_back(colorVector);

		}

	}
	return std::make_tuple(cloud, TransF * T265toLACCTransform);
}

std::tuple<double, double, double> RealsenseControl::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
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

void RealsenseControl::SLAM_RT()
{
	std::shared_ptr<open3d::geometry::PointCloud > CombinedCloud_ptr(new open3d::geometry::PointCloud);
	std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr(new open3d::geometry::PointCloud);

	double ptCloud_Voxel = 0.05;

	Eigen::Matrix4d T265toLACCTransform;
	T265toLACCTransform << -1, 0, 0, 0,
							0, 1, 0, 0,
							0, 0, -1, 0,
							0, 0, 0, 1;

	int i = 1;
	while (m_IsSLAMON)
	{
		if (!m_QueueRealtimeCloudTrans.empty())
		{
			auto pointPose = m_QueueRealtimeCloudTrans.front();
			m_QueueRealtimeCloudTrans.pop();

			if (i > 0) {
				auto TempPCL_T265 = std::get<0>(pointPose);
				Eigen::Matrix4d TransMat_init = std::get<1>(pointPose);


				if (!(*CombinedCloud_ptr).HasPoints())
				{
					*CombinedCloud_ptr = (*TempPCL_T265.VoxelDownSample(ptCloud_Voxel)).Transform(T265toLACCTransform).Transform(TransMat_init);
					m_SLAMEDPointCloud = *CombinedCloud_ptr;
				}
				else
				{
					std::cerr << "resgitration start" << std::endl;
					open3d::pipelines::registration::RegistrationResult regResult;

					*pointcloud_ptr = *TempPCL_T265.VoxelDownSample(ptCloud_Voxel);
					*CombinedCloud_ptr = *CombinedCloud_ptr + (*pointcloud_ptr).Transform(T265toLACCTransform).Transform(TransMat_init);
					m_SLAMEDPointCloud=*CombinedCloud_ptr;

				};

			}

		}

	}
}






void RealsenseControl::StartPipeLine()
{
	auto timeCheck = std::chrono::high_resolution_clock::now();
	rs2::pose_frame pose_frame(nullptr);
	Eigen::Matrix4d T265toLACCTransform;
	T265toLACCTransform << -1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;

	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
	thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1);
	thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 4.0);
	spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
	spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
	temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0);


	while (m_IsPipeLineOn)
	{


		for (auto&& pipe : pipelines) // loop over pipelines
		{
			auto frames = pipe.wait_for_frames();

			auto color = frames.get_color_frame();
			auto depth = frames.get_depth_frame();
			auto pose = frames.get_pose_frame();

			if (color) {
				CurrentVideoFrame = &color;
				pc.map_to(color);
			}
			if (depth) {
				//System::Diagnostics::Debug::WriteLine("Frame");
				rs2::frame depth2 = depth;
				depth2 = dec_filter.process(depth2);
				depth2 = thr_filter.process(depth2);
				depth2 = spat_filter.process(depth2);
				depth2 = temp_filter.process(depth2);
				points = pc.calculate(depth2);
			}
			if (pose) {
				pose_frame = pose;
			}

			if (points && pose_frame && color)
			{
				rs2_pose tempPoseData = pose_frame.get_pose_data();
				posedata = tempPoseData;
				rs2::pose_frame null_pose_frame(nullptr);
				pose_frame = null_pose_frame;
				int trackerConfidence = tempPoseData.tracker_confidence;


				//printf("tracker_confidence is %d. iter: %d \n ", tempPoseData.tracker_confidence, i);
				RealTimeCloudPose = PCL_Conversion(points, color, tempPoseData);
				m_RTPointCloud = std::get<0>(RealTimeCloudPose).Transform(T265toLACCTransform);
				std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - timeCheck; //as second
				if (m_IsSLAMON &&tempPoseData.tracker_confidence > 2 && duration.count() > 1.5)
				{
					timeCheck = std::chrono::high_resolution_clock::now();

					m_QueueRealtimeCloudTrans.push(RealTimeCloudPose);
					printf("Push QUEUE");
				}
			}
		}
	}
}


