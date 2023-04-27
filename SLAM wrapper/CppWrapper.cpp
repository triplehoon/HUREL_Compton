#include "LahgiControl.h"
#include "RtabmapSlamControl.h"
#include "CppWrapper.h"

using namespace HUREL::Compton;

sBitMapUnmanged GetCvToPointers(cv::Mat color, uint8_t** outPoint)
{

	sBitMapUnmanged outStruct{ *outPoint, 0, 0 , 0, 0 };


	if (outStruct.ptr != nullptr)
	{
		delete[] outStruct.ptr;
	}

	int imagesize = 0;


	if (color.cols == 0)
	{
		return outStruct;
	}
	outStruct.width = color.cols;
	outStruct.height = color.rows;
	outStruct.step = color.step;
	outStruct.channelSize = color.channels();
	imagesize = outStruct.width * outStruct.height * outStruct.channelSize;

	//RtabmapCppWrapper::instance().UnlockVideoFrame();
	outStruct.ptr = new uchar[imagesize];
	memcpy(outStruct.ptr, color.data, imagesize);

	return outStruct;
}


LahgiCppWrapper& HUREL::Compton::LahgiCppWrapper::instance()
{
	static LahgiCppWrapper& inst = *(new LahgiCppWrapper());
	static bool isLogHandled = false;
	if (!isLogHandled)
	{
		HUREL::Logger::Instance().Handle(HUREL::Compton::WrapperCaller::Logging);
		isLogHandled = true;
	}
	return inst;
}

bool HUREL::Compton::LahgiCppWrapper::SetType(eModuleCppWrapper type)
{
	HUREL::Compton::eMouduleType moduleType = HUREL::Compton::eMouduleType::MONO;
	switch (type)
	{
	case HUREL::Compton::eModuleCppWrapper::MONO:
		moduleType = HUREL::Compton::eMouduleType::MONO;
		break;
	case HUREL::Compton::eModuleCppWrapper::QUAD:
		moduleType = HUREL::Compton::eMouduleType::QUAD;

		break;
	case HUREL::Compton::eModuleCppWrapper::QUAD_DUAL:
		moduleType = HUREL::Compton::eMouduleType::QUAD_DUAL;
		break;
	default:
		break;
	}
	return LahgiControl::instance().SetType(moduleType);
}

void HUREL::Compton::LahgiCppWrapper::SetEchks(std::vector<std::vector<double>> echks)
{
	std::vector<sEnergyCheck> eChkUnmanagedVector;
	eChkUnmanagedVector.reserve(echks.size());
	for (auto echk : echks)
	{
		sEnergyCheck eChkUnmanaged;
		eChkUnmanaged.minE = echk[0];
		eChkUnmanaged.maxE = echk[1];

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}
	LahgiControl::instance().SetEchk(eChkUnmanagedVector);
}

void HUREL::Compton::LahgiCppWrapper::AddListModeDataWithTransformation(const unsigned short* byteData)
{

	LahgiControl::instance().AddListModeDataWithTransformation(byteData);
}

std::vector<ListModeDataCppWrapper> HUREL::Compton::LahgiCppWrapper::GetRelativeListModeData()
{
	std::vector<ListModeData> lists = LahgiControl::instance().GetListedListModeData();
	std::vector<ListModeDataCppWrapper> cppLists;
	for (auto lm : lists)
	{
		ListModeDataCppWrapper lmCpp;
		lmCpp.ScatterRelativeInteractionPointX = lm.Scatter.RelativeInteractionPoint[0];
		lmCpp.ScatterRelativeInteractionPointY = lm.Scatter.RelativeInteractionPoint[1];
		lmCpp.ScatterRelativeInteractionPointZ = lm.Scatter.RelativeInteractionPoint[2];
		lmCpp.ScatterInteractionEnergy = lm.Scatter.InteractionEnergy;
		lmCpp.AbsorberRelativeInteractionPointX = lm.Absorber.RelativeInteractionPoint[0];
		lmCpp.AbsorberRelativeInteractionPointY = lm.Absorber.RelativeInteractionPoint[1];
		lmCpp.AbsorberRelativeInteractionPointZ = lm.Absorber.RelativeInteractionPoint[2];
		lmCpp.AbsorberInteractionEnergy = lm.Absorber.InteractionEnergy;
		cppLists.push_back(lmCpp);
	}
	return cppLists;
}

size_t HUREL::Compton::LahgiCppWrapper::GetListedListModeDataSize()
{
	return LahgiControl::instance().GetListedListModeDataSize();
}


void HUREL::Compton::LahgiCppWrapper::ResetListedListModeData()
{
	LahgiControl::instance().ResetListedListModeData();
	for (int i = 0; i < 16; ++i)
	{
		LahgiControl::instance().ResetEnergySpectrum(i);
	}
}

void HUREL::Compton::LahgiCppWrapper::RestEnergySpectrum(int channelNumber)
{
	LahgiControl::instance().ResetEnergySpectrum(channelNumber);
}

std::tuple<double, double, double> HUREL::Compton::LahgiCppWrapper::GetEcalValue(int fpgaChannelNumber)
{
	return LahgiControl::instance().GetEcalValue(fpgaChannelNumber);
}

void HUREL::Compton::LahgiCppWrapper::SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal)
{
	LahgiControl::instance().SetEcalValue(fpgaChannelNumber, ecal);
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSpectrum(int channelNumber)
{
	return LahgiControl::instance().GetEnergySpectrum(channelNumber).GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetSumSpectrum()
{
	return LahgiControl::instance().GetSumEnergySpectrum().GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetAbsorberSumSpectrum()
{
	return LahgiControl::instance().GetAbsorberSumEnergySpectrum().GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetScatterSumSpectrum()
{
	return  LahgiControl::instance().GetScatterSumEnergySpectrum().GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetScatterSumSpectrum(int time)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData(time * 1000);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());



	int reconStartIndex = 0;


	EnergySpectrum spectClass = EnergySpectrum(10, 3000);;
	for (int i =0; i < lmData.size(); ++i)
	{
		if (lmData[i].InteractionChannel < 8)
		{
			spectClass.AddEnergy(lmData[i].Energy);
		}		
	}

	return spectClass.GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetAbsorberSumSpectrum(int time)
{
	std::vector<EnergyTimeData> lmData = LahgiControl::instance().GetListedEnergyTimeData(time * 1000);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());



	int reconStartIndex = 0;


	EnergySpectrum spectClass = EnergySpectrum(10, 3000);;
	for (int i = 0; i < lmData.size(); ++i)
	{
		if (lmData[i].InteractionChannel >= 8)
		{
			spectClass.AddEnergy(lmData[i].Energy);
		}
	}


	return spectClass.GetHistogramEnergy();
}

bool HUREL::Compton::LahgiCppWrapper::SaveListedListModeData(std::string filePath)
{
	LahgiControl::instance().SaveListedListModeData(filePath);
	return true;
}

bool HUREL::Compton::LahgiCppWrapper::LoadListedListModeData(std::string filePath)
{
	return LahgiControl::instance().LoadListedListModeData(filePath);
}

sBitMapUnmanged  HUREL::Compton::LahgiCppWrapper::GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
{
	static uint8_t* ptr = nullptr;

	return GetCvToPointers(LahgiControl::instance().GetResponseImage(imgSize, pixelCount, timeInSeconds, isScatter), &ptr);
}

std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  HUREL::Compton::LahgiCppWrapper::GetRadiation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion)
{
	static uint8_t* ptrCoded = nullptr;
	static uint8_t* ptrCompton = nullptr;
	static uint8_t* ptrHybrid = nullptr;

	RadiationImage radimage(LahgiControl::instance().GetListedListModeData(timeInMiliSeconds), s2M, det_W, resImprov, m2D, hFov, wFov);

	return std::make_tuple(GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mCodedImage, imgSize, minValuePortion), &ptrCoded), 
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mComptonImage, imgSize, minValuePortion), &ptrCompton),
		GetCvToPointers(RadiationImage::GetCV_32SAsJet(radimage.mHybridImage, imgSize, minValuePortion), &ptrHybrid));

}

sBitMapUnmanged HUREL::Compton::LahgiCppWrapper::GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution)
{
	static uint8_t* ptrImg = nullptr;
	
	Eigen::Matrix4d deviceTransformation = LahgiControl::instance().t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry()
										   * LahgiControl::instance().t265toLACCPosTransformInv * LahgiControl::instance().t265toLACCPosTranslate;

	cv::Mat p3 = RtabmapSlamControl::instance().GetCurrentPointsFrame(resolution);
	cv::Mat xyz[3];
	cv::split(p3, xyz);
	std::vector<ListModeData> data = LahgiControl::instance().GetListedListModeData(timeInMiliSeconds);
	cv::Mat radImgReturn = cv::Mat::zeros(p3.rows, p3.cols, CV_32S);
	
	if (data.size() == 0)
	{
		return GetCvToPointers(radImgReturn, &ptrImg);
	}

	long long startTime = data[0].InteractionTimeInMili.count();

	int startIndex = 0;
	int endIndex = 0;
	for (int i = 0; i < data.size(); ++i)
	{
		if (startTime != data[i].InteractionTimeInMili.count())
		{
			endIndex = i - 1;

			Eigen::Matrix4d diffMatrix = data[startIndex].DetectorTransformation * deviceTransformation.inverse();
			Eigen::Quaterniond quaternino(diffMatrix.topLeftCorner<3, 3>());
			

			std::vector<ListModeData>::const_iterator first = data.begin() + startIndex;
			std::vector<ListModeData>::const_iterator last = data.begin() + endIndex + 1;
			std::vector<ListModeData> newVec(first, last);

			RadiationImage radImg = RadiationImage(newVec);
			cv::Mat tempRadImg = cv::Mat::zeros(p3.rows, p3.cols, CV_32S);

			for (int iPix = 0; iPix < radImgReturn.rows; ++iPix)
			{
				for (int jPix = 0; jPix < radImgReturn.cols; ++jPix)
				{
					Eigen::Vector3d point(xyz[0].at<float>(iPix, jPix), xyz[1].at<float>(iPix, jPix), xyz[2].at<float>(iPix, jPix));

					tempRadImg.at<int>(iPix, jPix) = radImg.OverlayValue(point, eRadiationImagingMode::COMPTON);
				}
			}

			radImgReturn += tempRadImg;
			startIndex = i;
		}
	}
	
	cv::Mat jetImage = RadiationImage::GetCV_32SAsJet(radImgReturn, minValuePortion);


	return GetCvToPointers(jetImage, &ptrImg);
}


bool HUREL::Compton::RtabmapCppWrapper::GetIsSlamPipeOn()
{
	return RtabmapSlamControl::instance().mIsSlamPipeOn;
}

bool HUREL::Compton::RtabmapCppWrapper::GetIsVideoStreamOn()
{
	return RtabmapSlamControl::instance().mIsVideoStreamOn;
}

bool HUREL::Compton::RtabmapCppWrapper::Initiate()
{
	return RtabmapSlamControl::instance().Initiate();
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetRTPointCloud()
{

	std::vector<ReconPointCppWrapper> points;
	return points;
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetRTPointCloudTransposed()
{
	return std::vector<ReconPointCppWrapper>();
}

void HUREL::Compton::RtabmapCppWrapper::StartVideoStream()
{
	RtabmapSlamControl::instance().StartVideoStream();
}

bool HUREL::Compton::RtabmapCppWrapper::StartSlamPipe()
{
	RtabmapSlamControl::instance().StartSlamPipe();
	return true;
}

void HUREL::Compton::RtabmapCppWrapper::StopVideoStream()
{
	RtabmapSlamControl::instance().StopVideoStream();
}

void HUREL::Compton::RtabmapCppWrapper::StopSlamPipe()
{
	RtabmapSlamControl::instance().StopSlamPipe();
}

void HUREL::Compton::RtabmapCppWrapper::ResetSlam()
{
	RtabmapSlamControl::instance().ResetSlam();
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetSlamPointCloud()
{
	open3d::geometry::PointCloud  pc = RtabmapSlamControl::instance().GetSlamPointCloud();

	std::vector<ReconPointCppWrapper> returnPc;
	returnPc.reserve(pc.colors_.size());

	for (int i = 0; i < pc.colors_.size(); ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = pc.points_[i][0];
		tmpPoint.pointY = pc.points_[i][1];
		tmpPoint.pointZ = pc.points_[i][2];


		tmpPoint.colorR = pc.colors_[i][0];
		tmpPoint.colorG = pc.colors_[i][0];
		tmpPoint.colorB = pc.colors_[i][0];
		tmpPoint.colorA = 1;
		returnPc.push_back(tmpPoint);
	}

	return returnPc;
}

bool HUREL::Compton::RtabmapCppWrapper::LoadPlyFile(std::string filePath)
{
	return RtabmapSlamControl::instance().LoadPlyFile(filePath);
}

void HUREL::Compton::RtabmapCppWrapper::SavePlyFile(std::string filePath)
{
	open3d::geometry::PointCloud  pc = RtabmapSlamControl::instance().GetSlamPointCloud();
	open3d::io::WritePointCloudOption option;

	open3d::io::WritePointCloudToPLY(filePath, pc, option);
	
	
	cv::imwrite(filePath + "_depth.png", RtabmapSlamControl::instance().GetCurrentDepthFrame());
	cv::imwrite(filePath + "_rgb.png", RtabmapSlamControl::instance().GetCurrentVideoFrame());
}

std::vector<double> HUREL::Compton::RtabmapCppWrapper::getMatrix3DOneLineFromPoseData()
{
	return RtabmapSlamControl::instance().getMatrix3DOneLineFromPoseData();;
}

bool HUREL::Compton::RtabmapCppWrapper::GetCurrentVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStride, int* outChannelSize)
{
	static int imagesize = 0;
	static uchar* currentImg = nullptr;
	if (currentImg != nullptr)
	{
		delete[] currentImg;
		currentImg = nullptr;
	}
	cv::Mat color = RtabmapSlamControl::instance().GetCurrentVideoFrame();
	if (color.cols <= 0)
	{
		*outImgPtr = nullptr;
		return false;
	}
	*outWidth = color.cols;
	*outHeight = color.rows;
	*outStride = color.step;
	*outChannelSize = color.channels();
	if (imagesize != *outWidth * *outHeight * *outChannelSize)
	{
		imagesize = *outWidth * *outHeight * *outChannelSize;
		delete[] mColorImg;

	}
	currentImg = new uchar[imagesize];
	memcpy(currentImg, color.data, imagesize);
	//RtabmapCppWrapper::instance().UnlockVideoFrame();
	*outImgPtr = currentImg;
	return true;
}

std::vector<ReconPointCppWrapper>  HUREL::Compton::RtabmapCppWrapper::GetReconSLAMPointCloud(double time, eReconCppWrapper reconType, double voxelSize, bool useLoaded)
{
	open3d::geometry::PointCloud reconPC;
	if (useLoaded)
	{
		reconPC = RtabmapSlamControl::instance().GetLoadedPointCloud();
	}
	else
	{
		reconPC = RtabmapSlamControl::instance().GetSlamPointCloud();
	}

	open3d::geometry::PointCloud reconPcDownSampled = *reconPC.VoxelDownSample(voxelSize);

	ReconPointCloud o3dPc = LahgiControl::instance().GetReconOverlayPointCloudHybrid(reconPcDownSampled, time);


	std::vector<ReconPointCppWrapper> pc;

	int pcSize = 0;
	if (o3dPc.points_.size() > o3dPc.colors_.size())
	{
		pcSize = o3dPc.colors_.size();
	}
	else
	{
		pcSize = o3dPc.points_.size();
	}
	pc.reserve(pcSize);

	

	for (int i = 0; i < pcSize; ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = o3dPc.points_[i][0];
		tmpPoint.pointY = o3dPc.points_[i][1];
		tmpPoint.pointZ = o3dPc.points_[i][2];

		RGBA_t rgb = ReconPointCloud::ColorScaleJet(o3dPc.reconValues_[i], o3dPc.maxReoconValue*0.7, o3dPc.maxReoconValue);

		tmpPoint.colorR = rgb.R;

		tmpPoint.colorG = rgb.G;
		tmpPoint.colorB = rgb.B;
		tmpPoint.colorA = rgb.A;

		pc.push_back(tmpPoint);
	}
	return pc;

}

std::vector<std::vector<double>> HUREL::Compton::RtabmapCppWrapper::GetOptimizedPoses()
{

	std::vector<Eigen::Matrix4d> poses = RtabmapSlamControl::instance().GetOptimizedPoses();
	std::vector<std::vector<double>> returnPoses;
	returnPoses.reserve(poses.size());
	for (int i = 0; i < poses.size(); ++i)
	{
		Eigen::Matrix4d m = poses[i];
		auto Matrix3Dtype = m.adjoint();
		std::vector<double> matrix3DOneLine;
		matrix3DOneLine.reserve(16);
		int idx = 0;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				matrix3DOneLine.push_back(Matrix3Dtype(i, j));
				idx++;
			}
		}
		returnPoses.push_back(matrix3DOneLine);
	}



	return returnPoses;
}


std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetLoadedPointCloud()
{
	std::vector<ReconPointCppWrapper> pc;

	open3d::geometry::PointCloud o3dPc = RtabmapSlamControl::instance().GetLoadedPointCloud();
	int pcSize = 0;
	if (o3dPc.points_.size() > o3dPc.colors_.size())
	{
		pcSize = o3dPc.colors_.size();
	}
	else
	{
		pcSize = o3dPc.points_.size();
	}
	pc.reserve(pcSize);
	for (int i = 0; i < pcSize; ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = o3dPc.points_[i][0];
		tmpPoint.pointY = o3dPc.points_[i][1];
		tmpPoint.pointZ = o3dPc.points_[i][2];

		tmpPoint.colorR = o3dPc.colors_[i][0];

		tmpPoint.colorG = o3dPc.colors_[i][1];
		tmpPoint.colorB = o3dPc.colors_[i][2];
		tmpPoint.colorA = 1;
		pc.push_back(tmpPoint);
	}

	return pc;
}

RtabmapCppWrapper& HUREL::Compton::RtabmapCppWrapper::instance()
{
	static RtabmapCppWrapper& inst = *(new RtabmapCppWrapper());

	return inst;
}