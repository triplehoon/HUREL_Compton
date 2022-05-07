#include "RtabmapSlamControl.h"
#include <future>
#include <mutex>

using namespace HUREL::Compton;


HUREL::Compton::RtabmapSlamControl::RtabmapSlamControl()
{

}

bool HUREL::Compton::RtabmapSlamControl::Initiate(std::string* outMessage)
{
mCamera = new rtabmap::CameraRealSense2();
	mCamera->setResolution(848, 480);

	const rtabmap::Transform test(0.006f, 0.0025f, 0.0f, 0.0f, 0.0f, 0.0f);
	mCamera->setDualMode(true, test);
	mCamera->setOdomProvided(true, false, true);
	mCamera->setImagesRectified(true);
	if (!mCamera->init(".", ""))
	{
		*outMessage = "RtabmapSlamControl: Initiate failed\n";
		std::cout << (*outMessage);
		mIsInitiate = false;
		return false;
	}
	else
	{
		*outMessage = "RtabmapSlamControl: Initiate sucess \n";
		std::cout << (*outMessage);
		mIsInitiate = true;
		return true;
	}	
}

RtabmapSlamControl& HUREL::Compton::RtabmapSlamControl::instance()
{
	static RtabmapSlamControl* instance = new RtabmapSlamControl();
	//std::cout << "instance: " << instance << std::endl;
	return *instance;
}

HUREL::Compton::RtabmapSlamControl::~RtabmapSlamControl()
{
	delete mCamera;
	delete mCameraThread;
}

static std::future<void> t1;

void HUREL::Compton::RtabmapSlamControl::StartVideoStream()
{
	mIsVideoStreamOn = true;
	std::cout << "RtabmapSlamControl start video stream\n";
	auto func = std::bind(&RtabmapSlamControl::VideoStream, this);
	t1 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopVideoStream()
{
	mIsVideoStreamOn = false;
	t1.get();

	std::cout << "RtabmapSlamControl stop video stream\n";
}

static std::mutex videoStreamMutex;
static std::mutex pcMutex;

void HUREL::Compton::RtabmapSlamControl::VideoStream()
{
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	while (mIsVideoStreamOn)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			auto img = data.imageRaw();
			auto imgDepth = data.depthOrRightRaw();
			videoStreamMutex.lock();
			if (img.cols > 0)
			{			
				mCurrentVideoFrame = img;
			}
			if (imgDepth.cols > 0)
			{
				mCurrentDepthFrame = imgDepth;
			}

			if (mOdoInit && mIsSlamPipeOn)
			{
				mCurrentOdometry = t265toLACCAxisTransform * mOdo->getPose().toEigen4d()* mInitOdo.inverse();
			}

			

			videoStreamMutex.unlock();
			pcMutex.lock();
			mRealtimePointCloud = *(rtabmap::util3d::cloudRGBFromSensorData(data, 4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.0f));
			pcMutex.unlock();
		}
	}

	mCameraThread->join(true);
	mCameraThread->kill();
	delete mCameraThread;
	mCameraThread = nullptr;
}


void HUREL::Compton::RtabmapSlamControl::ResetSlam()
{
	videoStreamMutex.lock();
	mOdo->reset();
	mOdoInit = false;	
	videoStreamMutex.unlock();
}


cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame()
{
	return mCurrentVideoFrame;
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentDepthFrame()
{
	videoStreamMutex.lock();
	cv::Mat tmp = mCurrentDepthFrame;
	videoStreamMutex.unlock();
	return tmp;
}

void HUREL::Compton::RtabmapSlamControl::LockVideoFrame()
{
	videoStreamMutex.lock();

}

void HUREL::Compton::RtabmapSlamControl::UnlockVideoFrame()
{
	videoStreamMutex.unlock();
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetRTPointCloud()
{
	pcMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mRealtimePointCloud;
	pcMutex.unlock();
	
	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{	
		if (isnan(tmp[i].x) || isinf(tmp[i].x))
		{
			continue;
		}
		Eigen::Vector3d color(tmp[i].r/255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector3d point(tmp[i].x, tmp[i].y, tmp[i].z);
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(point);
	}

	return tmpOpen3dPc.Transform(t265toLACCAxisTransform);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetRTPointCloudTransposed()
{
	pcMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mRealtimePointCloud;
	pcMutex.unlock();
	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		if (isnan(tmp[i].x) || isinf(tmp[i].x))
		{
			continue;
		}
		Eigen::Vector3d color(tmp[i].b / 255.0, tmp[i].g / 255.0, tmp[i].r / 255.0);
		Eigen::Vector3d point(tmp[i].x, tmp[i].y, tmp[i].z);
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(point);
	}

	return tmpOpen3dPc.Transform(GetOdomentry()*t265toLACCAxisTransform);
}

static std::future<void> t2;

void HUREL::Compton::RtabmapSlamControl::StartSlamPipe()
{
	if (!mIsVideoStreamOn || !mIsInitiate)
	{
		std::cout << "RtabmapSlamControl fail to start slam pipe\n";
	}

	if (mIsSlamPipeOn)
	{
		std::cout << "SLAM Pipe is already on\n";
		return;
	}
	mIsSlamPipeOn = true;
	std::cout << "RtabmapSlamControl start slam pipe\n";
	auto func = std::bind(&RtabmapSlamControl::SlamPipe, this);
	t2 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopSlamPipe()
{
	mIsSlamPipeOn = false;
	t2.get();

	std::cout << "RtabmapSlamControl stop slam pipe\n";
}

static std::mutex slamPipeMutex;

void HUREL::Compton::RtabmapSlamControl::SlamPipe()
{
	mOdo = rtabmap::Odometry::create();

	rtabmap::OdometryThread odomThread(mOdo);

	// Create RTAB-Map to process OdometryEvent
	rtabmap::Rtabmap* rtabmap = new rtabmap::Rtabmap();
	rtabmap->init();
	rtabmap::RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
		// Setup handlers
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	UEventsManager::createPipe(mCameraThread, &odomThread, "CameraEvent");

	// Let's start the threads
	rtabmapThread.start();
	odomThread.start();

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	while (mIsSlamPipeOn)
	{
		
		std::map<int, rtabmap::Signature> nodes;
		std::map<int, rtabmap::Transform> optimizedPoses;
		std::multimap<int, rtabmap::Link> links;
		if (!mOdoInit)
		{
			rtabmap->resetMemory();
			mInitOdo = t265toLACCAxisTransform * mOdo->getPose().toEigen4d();
			mOdoInit = true;
		}
		rtabmap->getGraph(optimizedPoses, links, true, true, &nodes, true, true, true, true);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int i = 0;
		for (std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
		{
			rtabmap::Signature node = nodes.find(iter->first)->second;

			// uncompress data
			node.sensorData().uncompressData();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.0f);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::vector<int> index;
			pcl::removeNaNFromPointCloud(*tmp, *tmpNoNaN, index);
			if (!tmpNoNaN->empty())
			{
				*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
			}
			++i;
			//pintf("iter %d \n", i);
			//tmpNoNaN.reset();
			//delete test;
		}
		slamPipeMutex.lock();
		mSlamedPointCloud = *cloud;
		slamPipeMutex.unlock();
		Sleep(0);



	}
	mOdoInit = false;
	mCurrentOdometry = Eigen::Matrix4d::Identity();
	mInitOdo = Eigen::Matrix4d::Identity();
	odomThread.join(true);
	rtabmapThread.join(true);
	odomThread.kill();
	rtabmapThread.kill();
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetSlamPointCloud()
{
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	slamPipeMutex.unlock();

	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		Eigen::Vector3d color(tmp[i].b/255.0, tmp[i].g / 255.0, tmp[i].r / 255.0);
		Eigen::Vector4d point(tmp[i].x, tmp[i].y, tmp[i].z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * point;
		if (transFormedpoint.y() > 0.6)
		{
			continue;
		}
		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);
	}
	return *tmpOpen3dPc.VoxelDownSample(0.02);
}

std::vector<double> HUREL::Compton::RtabmapSlamControl::getMatrix3DOneLineFromPoseData()
{
	Eigen::Matrix4d m = GetOdomentry();
	auto Matrix3Dtype = m.adjoint();
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

Eigen::Matrix4d HUREL::Compton::RtabmapSlamControl::GetOdomentry()
{
	videoStreamMutex.lock();
	Eigen::Matrix4d tmp = mCurrentOdometry;
	videoStreamMutex.unlock();
	return tmp;
}