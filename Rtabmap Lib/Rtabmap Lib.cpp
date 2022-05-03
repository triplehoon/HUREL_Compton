// Rtabmap Lib.cpp : Defines the functions for the static library.
//

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

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>

// TODO: This is an example of a library function
void fnRtabmapLib()
{
	rtabmap::CameraRealSense2* camera = new rtabmap::CameraRealSense2;
	const rtabmap::Transform test(0.006f, 0.0025f, 0.0f, 0.0f, 0.0f, 0.0f);
	camera->setDualMode(true, test);
	camera->setOdomProvided(true, false, true);
}
