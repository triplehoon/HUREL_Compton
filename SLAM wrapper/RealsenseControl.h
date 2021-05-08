#pragma once

#define D455_H_COLOR_SIZE (1280)
#define D455_V_COLOR_SIZE (720)
#define D455_H_DEPTH_SIZE (480)
#define D455_V_DEPTH_SIZE (270)

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
	std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d> PCL_Conversion(const rs2::points* points, const rs2::video_frame* color, const rs2_pose* pose, std::vector<Eigen::Vector2f>* uv);
	std::tuple<double, double, double> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
	std::queue<std::tuple<open3d::geometry::PointCloud >> m_QueueSLAMedCloudTrans;
	Eigen::Matrix4d T265toLACCTransform;



	rs2::pointcloud pc;
	rs2::context ctx;

	std::vector<rs2::pipeline> pipelines;
	rs2::pipeline pipeD455;
	rs2::config cfgD455;
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
	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> m_RTPointCloud;
	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> m_RTPointCloudTransposed;
	open3d::geometry::PointCloud m_SLAMEDPointCloud;
	open3d::geometry::PointCloud m_SLAMEDPointCloudDownSampled;


	bool m_IsPipeLineOn;
	bool m_IsSLAMON;

	RealsenseControl();
	~RealsenseControl();
	
	bool InitiateRealsense(std::string* message);

	void RealsensesPipeline();

	void SLAMPipeline();

	static std::vector<double> getMatrix3DOneLineFromPoseData(rs2_pose poseData);
};

