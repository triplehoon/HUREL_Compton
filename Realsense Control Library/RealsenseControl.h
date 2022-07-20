#pragma once


#include <mutex>

#define D455_H_COLOR_SIZE (1280)
#define D455_V_COLOR_SIZE (800)
#define D455_H_DEPTH_SIZE (848)
#define D455_V_DEPTH_SIZE (480)

#define D435_H_COLOR_SIZE (1280)
#define D435_V_COLOR_SIZE (720)
#define D435_H_DEPTH_SIZE (848)
#define D435_V_DEPTH_SIZE (480)

#define T265_TO_LAHGI_OFFSET_X (-0.05)
#define T265_TO_LAHGI_OFFSET_Y (-0.308)
#define T265_TO_LAHGI_OFFSET_Z (-0.05)
#define T265_To_Mask_OFFSET_X (T265_TO_LAHGI_OFFSET_X)
#define T265_To_Mask_OFFSET_Y (T265_TO_LAHGI_OFFSET_Y)
#define T265_To_Mask_OFFSET_Z (0.025)

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
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>
#include <librealsense2/hpp/rs_frame.hpp>

#include "SLAMRobustRecon.h"
class RealsenseControl
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	std::queue<PC_TRANSPOS_TUPLE> m_QueueRealtimeCloudTrans;
	static std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d, std::vector<Eigen::Vector2f>> PCL_Conversion(const rs2::points& points, const rs2::video_frame& color, const rs2_pose& pose);
	static std::tuple<double, double, double> RGB_Texture(const rs2::video_frame& texture, const rs2::texture_coordinate& Texture_XY);
	std::queue<std::tuple<open3d::geometry::PointCloud>> m_QueueSLAMedCloudTrans;
	Eigen::Matrix4d T265toLACCTransform;

	Eigen::Matrix4d mRTTransformation;
	Eigen::Matrix4d mRTTransformationTrue;

	rs2_pose m_Posedata = rs2_pose();
	cv::Mat m_CurrentVideoFrame;
	cv::Mat m_CurrentDepthFrame;

	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> m_RTPointCloud;
	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> m_RTPointCloudTransposed;
	open3d::geometry::PointCloud m_SLAMEDPointCloud;
	open3d::geometry::PointCloud m_SLAMEDPointCloudDownSampled;

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
	RealsenseControl();

public:
	
	rs2_pose GetPoseData();
	cv::Mat GetCurrentVideoFrame();
	cv::Mat GetCurrentDepthFrame();

	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> GetRTPointCloud();
	std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> GetRTPointCloudTransposed();
	open3d::geometry::PointCloud GetSLAMEDPointCloud();
	open3d::geometry::PointCloud GetSLAMEDPointCloudDownSampled();


	bool IsPipeLineOn;
	bool IsSLAMON;

	bool SaveRTPointCloud(std::string& fileName);
	bool SaveSLAMEDPointCloud(std::string& fileName);
	bool SaveDownSampledSLAMEDPointCloud(std::string& fileName);

	~RealsenseControl();

	bool InitiateRealsense(std::string* outMessage);

	void RealsensesPipeline();

	void SLAMPipeline();

	static std::vector<double> getMatrix3DOneLineFromPoseData(rs2_pose poseData);

	Eigen::Matrix4d GetPoseDataEigen();

public:
	static RealsenseControl& instance();
};

