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

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

namespace HUREL
{
	namespace Compton
	{

		class RtabmapSlamControl
		{
		public:
			bool Initiate(std::string* outMessage);

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Eigen::Matrix4d GetOdomentry();
			cv::Mat GetCurrentVideoFrame();
			cv::Mat GetCurrentDepthFrame();
			std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> GetRTPointCloud();
			std::tuple<open3d::geometry::PointCloud, std::vector<Eigen::Vector2f>> GetRTPointCloudTransposed();

		public:
			static RtabmapSlamControl& instance();
		};


	};
};
