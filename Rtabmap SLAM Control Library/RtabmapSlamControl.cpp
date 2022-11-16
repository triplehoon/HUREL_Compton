#include "RtabmapSlamControl.h"
#include <future>
#include <mutex>

using namespace HUREL::Compton;



void ShowCV_32FAsJet(cv::Mat img, int size)
{
	if (img.type() != CV_32F)
	{
		return;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<float>(i, j)) - minValue)
				/ (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::Mat showImg;

	int sizeHeight = size;
	int sizeWidth = size;

	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST_EXACT);
	cv::imshow("img", showImg);
	cv::waitKey(0);
}


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

		const rtabmap::Transform test(0.000f, 0.13f, 0.0f, 0.0f, 0.0f, 0.0f);
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
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl start video stream", eLoggerType::INFO);
	
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	mIsVideoStreamOn = true;


	//auto func = std::bind(&RtabmapSlamControl::VideoStream, this);
	//t1 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopVideoStream()
{
	if (!mIsVideoStreamOn)
	{
		return;
	}
	mIsVideoStreamOn = false;
	
	mCameraThread->join(true);
	mCameraThread->kill();
	delete mCameraThread;
	mCameraThread = nullptr;

	//t1.get();
	//HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl stop video stream", eLoggerType::INFO);
	
}

static std::mutex videoStreamMutex;
static std::mutex pcMutex;


cv::Mat ConvertDepthTo3DPoint(cv::Mat& depthI, float fx, float fy, float cx, float cy)
{
	//float x,y,z
	cv::Mat points, chans[3];

	static int rows = 0;
	static int cols = 0;
	static cv::Mat uMat, vMat;
	cv::Mat depth;
	depthI.convertTo(depth, CV_32FC1);

	if (rows != depth.rows || cols != depth.cols)
	{
		rows = depth.rows;
		cols = depth.cols;
		if (rows == 0 || cols == 0)
		{
			return points;
		}

		uMat = cv::Mat(depth.rows, depth.cols, CV_32FC1);
		vMat = cv::Mat(depth.rows, depth.cols, CV_32FC1);

		for (int u = 0; u < rows; ++u)
		{
			uMat.row(u).setTo(u);
		}
		for (int v = 0; v < cols; ++v)
		{
			vMat.col(v).setTo(v);
		}
	}
	

	cv::Mat x_over_z = (cx - uMat) /fx;
	cv::Mat y_over_z = (cy - vMat) / fy;
	cv::Mat sqrtVlaue;
	ShowCV_32FAsJet(x_over_z, 600);
	ShowCV_32FAsJet(y_over_z, 600);
	cv::Mat before = 1 + x_over_z.mul(x_over_z) + y_over_z.mul(y_over_z);
	cv::sqrt(before, sqrtVlaue);
	chans[2] = depth.mul(1 / sqrtVlaue);
	chans[1] = x_over_z .mul(depth);
	chans[0] = y_over_z.mul(depth);

	cv::merge(chans, 3, points);

	return points;
}

void HUREL::Compton::RtabmapSlamControl::VideoStream()
{
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	mIsVideoStreamOn = true;

	while (mIsVideoStreamOn)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		
		if (data.isValid())
		{
			
			auto img = data.imageRaw();
			auto imgDepth = data.depthOrRightRaw();
			float fxValue = static_cast<float>(data.stereoCameraModels()[0].left().fx());
			float fyValue = static_cast<float>(data.stereoCameraModels()[0].left().fy());
			float cxValue = static_cast<float>(data.stereoCameraModels()[0].left().cx());
			float cyValue = static_cast<float>(data.stereoCameraModels()[0].left().cy());
			
			videoStreamMutex.lock();
			if (img.cols > 0)
			{			
				mCurrentVideoFrame = img;
			}
			if (imgDepth.cols > 0)
			{
				mCurrentDepthFrame = imgDepth;
				cv::Mat point3 = ConvertDepthTo3DPoint(imgDepth, fxValue, fyValue, cxValue, cyValue);
			}
			if (mOdo != nullptr && &mOdo->getPose() != nullptr && mIsSlamPipeOn == true )
			{
			
				//mCurrentOdometry = mOdo->getPose().toEigen4d() * t265toLACCAxisTransform;
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
	if (mOdo != nullptr)
	{
		mOdo->reset();
	}
	mOdoInit = false;	
	videoStreamMutex.unlock();
}


cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame()
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			img = data.imageRaw();

			float fxValue = static_cast<float>(data.cameraModels()[0].fx());
			float fyValue = static_cast<float>(data.cameraModels()[0].fy());
			float cxValue = static_cast<float>(data.cameraModels()[0].cx());
			float cyValue = static_cast<float>(data.cameraModels()[0].cy());
			cv::Mat dImg = data.depthRaw();

			if (dImg.cols > 0)
			{
				cv::Mat point3 = ConvertDepthTo3DPoint(dImg, fxValue, fyValue, cxValue, cyValue);
			}
		}
	}
	return img;
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentDepthFrame()
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			img = data.depthRaw();
		}
	}
	return img;
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentPointsFrame()
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{


			float fxValue = static_cast<float>(data.cameraModels()[0].fx());
			float fyValue = static_cast<float>(data.cameraModels()[0].fy());
			float cxValue = static_cast<float>(data.cameraModels()[0].cx());
			float cyValue = static_cast<float>(data.cameraModels()[0].cy());
			cv::Mat dImg = data.depthRaw();

			if (dImg.cols > 0)
			{
				img = ConvertDepthTo3DPoint(dImg, fxValue, fyValue, cxValue, cyValue);
			}
		}
	}
	return img;
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
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl start slam pipe", eLoggerType::INFO);
	auto func = std::bind(&RtabmapSlamControl::SlamPipe, this);
	t2 = std::async(std::launch::async, func);

}

void HUREL::Compton::RtabmapSlamControl::StopSlamPipe()
{
	if (mIsSlamPipeOn)
	{
		mIsSlamPipeOn = false;

		t2.get();
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl stop slam pipe", eLoggerType::INFO);
	}
	else {
		HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl is alread stop", eLoggerType::INFO);

	}
	

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
	mIsSlamPipeOn = true;

	while (mIsSlamPipeOn)
	{
		
		std::map<int, rtabmap::Signature> nodes;
		std::map<int, rtabmap::Transform> optimizedPoses;
		std::multimap<int, rtabmap::Link> links;
		if (!mOdoInit)
		{
			videoStreamMutex.lock();
			rtabmap->resetMemory();
			mInitOdo = t265toLACCAxisTransform* mOdo->getPose().toEigen4d() ;
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
			tempPoses.push_back(t265toLACCAxisTransform * iter->second.toEigen4d()  );
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
	//mCurrentOdometry = Eigen::Matrix4d::Identity();
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
	
		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);
	}

	std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	std::string fileName = std::to_string(timeInMili.count());
	open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.05);
	open3d::io::WritePointCloudOption option;
	open3d::io::WritePointCloudToPLY(fileName + ".ply", returnPC, option);


	cv::imwrite(fileName + "_depth.png", RtabmapSlamControl::instance().GetCurrentDepthFrame());
	cv::imwrite(fileName + "_rgb.png", RtabmapSlamControl::instance().GetCurrentVideoFrame());



	return returnPC;
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
	Eigen::Matrix4d odo = Eigen::Matrix4d::Identity();;
	
	//videoStreamMutex.lock();
	if (mOdo != nullptr && &mOdo->getPose() != nullptr && mIsSlamPipeOn == true)
	{
		odo =  mOdo->getPose().toEigen4d() ;
	}
	//videoStreamMutex.unlock();
	return odo;
}

std::vector<Eigen::Matrix4d> HUREL::Compton::RtabmapSlamControl::GetOptimizedPoses()
{

	slamPipeMutex.lock();
	std::vector<Eigen::Matrix4d> tempPoses = mPoses;
	slamPipeMutex.unlock();
	return tempPoses;
}
