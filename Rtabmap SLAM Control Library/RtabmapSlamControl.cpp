#include "RtabmapSlamControl.h"
#include <future>
#include <mutex>

using namespace HUREL::Compton;


HUREL::Compton::RtabmapSlamControl::RtabmapSlamControl()
{

}

bool HUREL::Compton::RtabmapSlamControl::Initiate()
{
	std::string msg;
	std::string* outMessage = &msg;
	try {
		rs2::context ctx = rs2::context();
		rs2::device_list devs =  ctx.query_devices();
		
		rs2::config cfgD455 = rs2::config();
		int devCounts = devs.size();
		if (devCounts == 0)
		{
			*outMessage += "No cameras";
			HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);
			return false;
		}
		std::string usbInfo;
		for (int i = 0; i < devCounts; ++i)
		{
			std::string devInof = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
			if (devInof == "Intel RealSense D455")
			{
				usbInfo = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
			}
		}
		//cfgD455.enable_device("935322071433");	

		*outMessage = "RtabmapSlamControl: d455 connencted with usb " + usbInfo + " \n";

	}
	catch (const rs2::camera_disconnected_error& e)
	{
		*outMessage += "RtabmapSlamControl: Camera was disconnected! Please connect it back ";
		*outMessage += e.what();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		return false;
	}
	// continue with more general cases
	catch (const rs2::recoverable_error& e)
	{
		*outMessage += "RtabmapSlamControl: Operation failed, please try again ";
		*outMessage +=  e.what();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		return false;
	}
	// you can also catch "anything else" raised from the library by catching rs2::error
	catch (const rs2::error& e)
	{
		*outMessage += "RtabmapSlamControl: Some other error occurred! ";
		*outMessage += e.what();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		return false;
	}
	
	try
	{
		mCamera = new rtabmap::CameraRealSense2();

		mCamera->setResolution(848, 480);
		mCamera->setDepthResolution(848, 480);

		const rtabmap::Transform test(0.006f, 0.0025f, 0.0f, 0.0f, 0.0f, 0.0f);
		mCamera->setDualMode(true, test);

		//mCamera->setOdomProvided(true, false, true);
		//mCamera->setImagesRectified(true);
	}
	catch(int exp)
	{
		*outMessage += "RtabmapSlamControl: CameraRealSense2 error " + exp ;
	}
	if (!mCamera->init(".", ""))
	{
		*outMessage += "RtabmapSlamControl: Initiate failed";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::ERROR_t);

		mIsInitiate = false;
		return false;
	}
	else
	{
		*outMessage += "RtabmapSlamControl: Initiate success";
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", *outMessage, eLoggerType::INFO);
		StartVideoStream();
		mIsInitiate = true;
		return true;
	}	
}

RtabmapSlamControl& HUREL::Compton::RtabmapSlamControl::instance()
{
	static RtabmapSlamControl* instance = new RtabmapSlamControl();
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
	if (!mIsInitiate || mIsVideoStreamOn)
	{
		return;
	}
	mIsVideoStreamOn = true;
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl start video stream", eLoggerType::INFO);
	
	auto func = std::bind(&RtabmapSlamControl::VideoStream, this);
	t1 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopVideoStream()
{
	mIsVideoStreamOn = false;
	t1.get();
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl stop video stream", eLoggerType::INFO);
	
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
			if (mOdo != nullptr && mIsSlamPipeOn == true)
			{
			
				mCurrentOdometry = mOdo->getPose().toEigen4d();
				videoStreamMutex.unlock();
			}
			else
			{
				videoStreamMutex.unlock();
			}
			
			
			//pcMutex.lock();
			//mRealtimePointCloud = *(rtabmap::util3d::cloudRGBFromSensorData(data, 4,           // image decimation before creating the clouds
			//	6.0f,        // maximum depth of the cloud
			//	0.5f));
			//pcMutex.unlock();
		}
	}

	mCameraThread->join(true);
	mCameraThread->kill();
	delete mCameraThread;
	mCameraThread = nullptr;
}


void HUREL::Compton::RtabmapSlamControl::ResetSlam()
{
	if (mOdo == nullptr)
	{
		return;
	}
	videoStreamMutex.lock();
	mOdo->reset();
	mOdoInit = false;	
	videoStreamMutex.unlock();
}


cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame()
{
	videoStreamMutex.lock();
	cv::Mat tmp = mCurrentVideoFrame.clone();
	videoStreamMutex.unlock();
	return tmp;
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentDepthFrame()
{
	videoStreamMutex.lock();
	cv::Mat tmp = mCurrentDepthFrame.clone();
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
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl fail to start slam pipe, Initiation failed", eLoggerType::ERROR_t);	}

	if (mIsSlamPipeOn)
	{
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "SLAM Pipe is already on", eLoggerType::INFO);
		
		return;
	}
	mIsSlamPipeOn = true;
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl start slam pipe", eLoggerType::INFO);
	auto func = std::bind(&RtabmapSlamControl::SlamPipe, this);
	t2 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopSlamPipe()
{
	mIsSlamPipeOn = false;
	t2.get();
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl stop slam pipe", eLoggerType::INFO);

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
	rtabmap::OccupancyGrid grid;

	while (mIsSlamPipeOn)
	{
		
		std::map<int, rtabmap::Signature> nodes;
		std::map<int, rtabmap::Transform> optimizedPoses;
		std::multimap<int, rtabmap::Link> links;
		if (!mOdoInit)
		{
			videoStreamMutex.lock();
			rtabmap->resetMemory();
			mInitOdo = t265toLACCAxisTransform * mOdo->getPose().toEigen4d();
			mOdoInit = true;
			videoStreamMutex.unlock();
			continue;
		}
		std::map<int, rtabmap::Signature> tmpnodes;
		std::map<int, rtabmap::Transform> tmpOptimizedPoses;
		
		rtabmap->getGraph(tmpOptimizedPoses, links, true, true, &tmpnodes, true, true, true, false);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int i = 0;
		rtabmap->getStatistics();

		nodes = tmpnodes;
		optimizedPoses = tmpOptimizedPoses;
		std::vector < Eigen::Matrix4d> tempPoses;
		tempPoses.reserve(optimizedPoses.size());
		//https://cpp.hotexamples.com/examples/-/Rtabmap/-/cpp-rtabmap-class-examples.html
		for (std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
		{
			
			if (nodes.count(iter->first) == 0)
			{
				continue;
			}
			rtabmap::Signature node = nodes.find(iter->first)->second;
			// uncompress data
			cv::Mat ground, obstacles, empty;
			//node.sensorData().uncompressData(0,0,0,0,&ground,&obstacles,&empty);
			node.sensorData().uncompressData();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.3f);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::vector<int> index;
			pcl::removeNaNFromPointCloud(*tmp,  *tmpNoNaN, index);
			//grid.addToCache(iter->first, ground, obstacles, empty);
			// 
			// 
			if (!tmpNoNaN->empty())
			{
				*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
			}
			tempPoses.push_back(iter->second.toEigen4d());
			++i;
			//pintf("iter %d \n", i);
			tmpNoNaN.reset();
			//delete test;
		}
		//grid.update(optimizedPoses);
		
		slamPipeMutex.lock();
		mSlamedPointCloud = *cloud;
		mPoses = tempPoses;
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
	if (!mIsSlamPipeOn)
	{
		return open3d::geometry::PointCloud();
	}
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
		Eigen::Vector3d color(tmp[i].r/255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector4d point(tmp[i].x, tmp[i].y, tmp[i].z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * point;
		if (transFormedpoint.y() > 0.7)
		{
			continue;
		}
		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);
	}
	return *tmpOpen3dPc.VoxelDownSample(0.05);
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

bool HUREL::Compton::RtabmapSlamControl::LoadPlyFile(std::string filePath)
{
	open3d::io::ReadPointCloudOption opt;
	return open3d::io::ReadPointCloudFromPLY(filePath, mLoadedPointcloud, opt);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetLoadedPointCloud()
{
	return mLoadedPointcloud;
}

Eigen::Matrix4d HUREL::Compton::RtabmapSlamControl::GetOdomentry()
{
	videoStreamMutex.lock();
	Eigen::Matrix4d tmp = mCurrentOdometry;
	videoStreamMutex.unlock();
	return tmp;
}

std::vector<Eigen::Matrix4d> HUREL::Compton::RtabmapSlamControl::GetOptimizedPoses()
{
	slamPipeMutex.lock();
	std::vector<Eigen::Matrix4d> tempPoses = mPoses;
	slamPipeMutex.unlock();
	return tempPoses;
}
