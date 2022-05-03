// Rtabmap SLAM.cpp : This file contains the 'main' function. Program execution begins and ends there.
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

using namespace rtabmap;
void eraseLines(int count) {
	if (count > 0) {
		std::cout << "\x1b[2K"; // Delete current line
		// i=1 because we included the first line
		for (int i = 1; i < count; i++) {
			std::cout
				<< "\x1b[1A" // Move cursor up one
				<< "\x1b[2K"; // Delete the entire line
		}
		std::cout << "\r"; // Resume the cursor at beginning of line
	}
}
int main()
{
    std::cout << "Hello World!\n";
    
    rtabmap::CameraRealSense2* camera = new rtabmap::CameraRealSense2;
	const rtabmap::Transform test(0.006f, 0.0025f, 0.0f, 0.0f, 0.0f, 0.0f);
	camera->setDualMode(true, test);
	camera->setOdomProvided(true, false, true);

	if (!camera->init("."))
	{
		std::cout << "Setting error \n";
	}
    CameraThread cameraThread(camera);
    Odometry* odo = Odometry::create();

    OdometryThread odomThread(odo);

    // Create RTAB-Map to process OdometryEvent
    Rtabmap* rtabmap = new Rtabmap();
    rtabmap->init();
    RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
        // Setup handlers
    odomThread.registerToEventsManager();
    rtabmapThread.registerToEventsManager();
    UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

    // Let's start the threads
    rtabmapThread.start();
    odomThread.start();
    cameraThread.start();
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
	    1, 0, 0, 0,
		0, 0, 0, 1;
    
    std::cout << camera->getSerial() << std::endl;
    while (1)
    {
		CameraInfo info;
		
		SensorData data = camera->takeImage();
		
		if (data.isValid())
		{
			
			auto img = data.imageRaw();
			if (img.cols > 0)
			{
				cv::imshow("", img); 
				eraseLines(5);
				std::cout << t265toLACCAxisTransform *odo->getPose().toEigen4d() << std::endl;
				
				if (cv::waitKey(100) > 0)
				{
					break;
				}
			}
		}
        
    }
	
    
    odomThread.join(true);
    rtabmapThread.join(true);
    odomThread.kill();
    rtabmapThread.kill();


	printf("Saving rtabmap_cloud.pcd...\n");
	std::map<int, Signature> nodes;
	std::map<int, Transform> optimizedPoses;
	std::multimap<int, Link> links;
	rtabmap->getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int i = 0;
	for (std::map<int, Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
	{
		Signature node = nodes.find(iter->first)->second;

		// uncompress data
		node.sensorData().uncompressData();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudRGBFromSensorData(
			node.sensorData(),
			4,           // image decimation before creating the clouds
			4.0f,        // maximum depth of the cloud
			0.0f);
		auto test = new pcl::PointCloud<pcl::PointXYZRGB>;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(test);
		std::vector<int> index;
		pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
		if (!tmpNoNaN->empty())
		{
			*cloud += *util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
		}		
		++i;
		printf("iter %d \n", i);
		delete test;
	}

	if (cloud->size())
	{
		printf("Voxel grid filtering of the assembled cloud (voxel=%f, %d points)\n", 0.01f, (int)cloud->size());
		cloud = util3d::voxelize(cloud, 0.01f);

		printf("Saving rtabmap_cloud.pcd... done! (%d points)\n", (int)cloud->size());
		pcl::io::savePCDFile("rtabmap_cloud.pcd", *cloud);
		pcl::io::savePLYFile("rtabmap_cloud.ply", *cloud); // to save in PLY format
	}


}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
