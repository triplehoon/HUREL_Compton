#pragma once

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OccupancyGrid.h"

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraThread.h"

#include <iostream>

#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>
#include <librealsense2/rs.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

#include "Logger.h"

#define T265_TO_LAHGI_OFFSET_X (-0.10)
#define T265_TO_LAHGI_OFFSET_Y (0.35)
#define T265_TO_LAHGI_OFFSET_Z (-0.05)
#define T265_To_Mask_OFFSET_X (T265_TO_LAHGI_OFFSET_X)
#define T265_To_Mask_OFFSET_Y (T265_TO_LAHGI_OFFSET_Y)
#define T265_To_Mask_OFFSET_Z (0.00)

namespace HUREL
{
	namespace Compton
	{

		class RtabmapSlamControl
		{

		private:
			
			Eigen::Matrix4d mInitOdo = Eigen::Matrix4d::Identity();

			rtabmap::CameraRealSense2* mCamera = nullptr;
			rtabmap::CameraThread* mCameraThread = nullptr;
			rtabmap::Odometry* mOdo = nullptr;// = rtabmap::Odometry::create();;
			
			cv::Mat mCurrentVideoFrame = cv::Mat();
			cv::Mat mCurrentDepthFrame = cv::Mat();
			void VideoStream();

			pcl::PointCloud<pcl::PointXYZRGB> mRealtimePointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB> mSlamedPointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
			Eigen::Matrix4d mCurrentOdometry = Eigen::Matrix4d::Identity();
			void SlamPipe();

			RtabmapSlamControl();

			open3d::geometry::PointCloud mLoadedPointcloud = open3d::geometry::PointCloud();
			std::vector < Eigen::Matrix4d> mPoses = std::vector<Eigen::Matrix4d>();
			void LockVideoFrame();

			void UnlockVideoFrame();
			void LockDepthFrame();

			void UnlockDepthFrame();
		public:
			bool mIsInitiate = false;
			bool mIsVideoStreamOn = false;
			bool mIsSlamPipeOn = false;
			bool mOdoInit = false;

			bool Initiate();

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Eigen::Matrix4d GetOdomentry();

			std::vector< Eigen::Matrix4d> GetOptimizedPoses();

			cv::Mat GetCurrentVideoFrame();
			cv::Mat GetCurrentDepthFrame();
			open3d::geometry::PointCloud GetRTPointCloud();
			open3d::geometry::PointCloud GetRTPointCloudTransposed();

			void StartVideoStream();
			void StopVideoStream();
			
			void StartSlamPipe();
			void StopSlamPipe();

			void ResetSlam();

			open3d::geometry::PointCloud GetSlamPointCloud();

			std::vector<double> getMatrix3DOneLineFromPoseData();

			bool LoadPlyFile(std::string filePath);

			open3d::geometry::PointCloud GetLoadedPointCloud();
		public:
			static RtabmapSlamControl& instance();
			~RtabmapSlamControl();
		};


	};
};
