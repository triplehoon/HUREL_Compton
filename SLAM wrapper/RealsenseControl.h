#pragma once


#include <librealsense2/rs.hpp>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <librealsense2/hpp/rs_frame.hpp>

#define NOMINMAX
#include <Windows.h>
#include <chrono>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <fstream>   
#include <cstdio>
#include <map>
#include <queue>
#include <math.h>



class RealsenseControl
{
private:
	std::queue<std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d>> m_QueueRealtimeCloudTrans;
	std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d> PCL_Conversion(const rs2::points& points, const rs2::video_frame& color, const rs2_pose& pose);
	std::tuple<double, double, double> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	std::queue<std::tuple<open3d::geometry::PointCloud >> m_QueueSLAMedCloudTrans;
	std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d> m_RealTimeCloudPose;
	Eigen::Matrix4d T265toLACCTransform;



	rs2::pointcloud pc;
	rs2::points points;
	rs2::context ctx;

	std::vector<rs2::pipeline> pipelines;
	rs2::pipeline pipeD435;
	rs2::config cfgD435;
	rs2::pipeline pipeT265;
	rs2::config cfgT265;
	rs2::decimation_filter dec_filter;
	rs2::threshold_filter thr_filter;
	rs2::spatial_filter spat_filter;
	rs2::temporal_filter temp_filter;
	rs2::hole_filling_filter hole_filter;


public:

	rs2_pose m_Posedata;
	rs2::video_frame m_CurrentVideoFrame = rs2::video_frame(nullptr);
	open3d::geometry::PointCloud m_RTPointCloud;
	open3d::geometry::PointCloud m_SLAMEDPointCloud;

	bool m_IsPipeLineOn;
	bool m_IsSLAMON;

	RealsenseControl();
	~RealsenseControl();
	
	bool InitiateRealsense(std::string* message);

	void RealsensesPipeline();

	void SLAMPipeline();

	static std::vector<double> getMatrix3DOneLineFromPoseData(rs2_pose poseData);
};

