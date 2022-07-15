#include "RtabmapSlamControl.h"
#include <future>
#include <mutex>

using namespace HUREL::Compton;


HUREL::Compton::RtabmapSlamControl::RtabmapSlamControl()
{

}

bool HUREL::Compton::RtabmapSlamControl::Initiate(std::string* outMessage)
{
	try {
		rs2::context ctx = rs2::context();

		rs2::config cfgD455 = rs2::config();
		//cfgD455.enable_device("935322071433");	
		cfgD455.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 15);
		//cfgD455.enable_stream(RS2_STREAM_COLOR, D435_H_COLOR_SIZE, D435_V_COLOR_SIZE, RS2_FORMAT_RGB8, 15);
		cfgD455.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 15);
		rs2::pipeline pipeD455 = rs2::pipeline();
		
		rs2::config cfgT265 = rs2::config();
		cfgT265.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		rs2::pipeline pipeT265 = rs2::pipeline(ctx);
		rs2::pipeline_profile d455 = pipeD455.start(cfgD455);

		std::string usbInfo = d455.get_device().get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
		*outMessage = "RtabmapSlamControl: d455 connencted with usb " + usbInfo + " \n";
		pipeT265.start(cfgT265);

		pipeD455.stop();
		pipeT265.stop();
	}
	catch (const rs2::camera_disconnected_error& e)
	{
		*outMessage += "RtabmapSlamControl: Camera was disconnected! Please connect it back ";
		*outMessage += e.what();
		return false;
	}
	// continue with more general cases
	catch (const rs2::recoverable_error& e)
	{
		*outMessage += "RtabmapSlamControl: Operation failed, please try again ";
		*outMessage +=  e.what();
		return false;
	}
	// you can also catch "anything else" raised from the library by catching rs2::error
	catch (const rs2::error& e)
	{
		*outMessage += "RtabmapSlamControl: Some other error occurred! ";
		*outMessage += e.what();
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
		std::cout << "RtabmapSlamControl: CameraRealSense2 error " << exp << std::endl;
	}
	if (!mCamera->init(".", ""))
	{
		*outMessage += "RtabmapSlamControl: Initiate failed\n";
		
		mIsInitiate = false;
		return false;
	}
	else
	{
		*outMessage += "RtabmapSlamControl: Initiate success\n";
		
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
	return mCurrentVideoFrame.clone();
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
			videoStreamMutex.lock();
			rtabmap->resetMemory();
			mInitOdo = t265toLACCAxisTransform * mOdo->getPose().toEigen4d();
			mOdoInit = true;
			videoStreamMutex.unlock();
			continue;
		}
		std::map<int, rtabmap::Signature> tmpnodes;
		std::map<int, rtabmap::Transform> tmpOptimizedPoses;
		
		rtabmap->getGraph(tmpOptimizedPoses, links, true, true, &tmpnodes, true, true, true, true);
		nodes = tmpnodes;
		optimizedPoses = tmpOptimizedPoses;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int i = 0;
		for (std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
		{

			if (nodes.count(iter->first) == 0)
			{
				continue;
			}
			rtabmap::Signature node = nodes.find(iter->first)->second;


			// uncompress data
			node.sensorData().uncompressData();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				4.0f,        // maximum depth of the cloud
				0.3f);
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

Eigen::Matrix4d HUREL::Compton::RtabmapSlamControl::GetOdomentry()
{
	videoStreamMutex.lock();
	Eigen::Matrix4d tmp = mCurrentOdometry;
	videoStreamMutex.unlock();
	return tmp;
}