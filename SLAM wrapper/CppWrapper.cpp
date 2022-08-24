#include "LahgiControl.h"
#include "RtabmapSlamControl.h"
#include "CppWrapper.h"

using namespace HUREL::Compton;

class test
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



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

void HUREL::Compton::LahgiCppWrapper::AddListModeDataWithTransformation(const unsigned short* byteData, std::vector<std::vector<double>>  echks)
{
	std::vector<sEnergyCheck> eChkUnmanagedVector;
	eChkUnmanagedVector.resize(echks.size());
	for(auto echk : echks)
	{
		sEnergyCheck eChkUnmanaged;
		eChkUnmanaged.minE = echk[0];
		eChkUnmanaged.maxE = echk[1];

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}

	LahgiControl::instance().AddListModeDataWithTransformation(byteData, eChkUnmanagedVector);
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

void HUREL::Compton::LahgiCppWrapper::RestListedListModeData()
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
	std::vector<ListModeData> lmData = LahgiControl::instance().GetListedListModeData();

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());



	int reconStartIndex = 0;


	EnergySpectrum spectClass = EnergySpectrum(5, 3000);;
	for (int i = lmData.size(); i--; i >= 0)
	{
		if (time != 0 && t.count() - lmData[i].InteractionTimeInMili.count() > static_cast<__int64>(time))
		{
			break;
		}

		spectClass.AddEnergy(lmData[i].Scatter.InteractionEnergy);
	}

	return spectClass.GetHistogramEnergy();
}

std::vector<BinningEnergy> HUREL::Compton::LahgiCppWrapper::GetAbsorberSumSpectrum(int time)
{
	std::vector<ListModeData> lmData = LahgiControl::instance().GetListedListModeData();

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());



	int reconStartIndex = 0;


	EnergySpectrum spectClass = EnergySpectrum(5, 3000);;
	for (int i = lmData.size(); i--; i >= 0)
	{
		if (t.count() - lmData[i].InteractionTimeInMili.count() > static_cast<__int64>(time))
		{
			break;
		}

		spectClass.AddEnergy(lmData[i].Absorber.InteractionEnergy);
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

bool HUREL::Compton::RtabmapCppWrapper::GetIsSlamPipeOn()
{
	return false;
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
}

bool HUREL::Compton::RtabmapCppWrapper::StartSlamPipe()
{
	return false;
}

void HUREL::Compton::RtabmapCppWrapper::StopVideoStream()
{
}

void HUREL::Compton::RtabmapCppWrapper::StopSlamPipe()
{
}

void HUREL::Compton::RtabmapCppWrapper::ResetSlam()
{
}

std::vector<ReconPointCppWrapper> HUREL::Compton::RtabmapCppWrapper::GetSlamPointCloud()
{
	open3d::geometry::PointCloud  pc =RtabmapSlamControl::instance().GetSlamPointCloud();

	std::vector<ReconPointCppWrapper> returnPc;
	returnPc.reserve(pc.colors_.size());

	for (int i = 0; i < pc.colors_.size(); ++i)
	{
		ReconPointCppWrapper tmpPoint;
		tmpPoint.pointX = o3dPc.points_[i][0];
		tmpPoint.pointY = o3dPc.points_[i][1];
		tmpPoint.pointZ = o3dPc.points_[i][2];


		RGBA_t rgb = ReconPointCloud::ColorScaleJet(o3dPc.reconValues_[i], 0, o3dPc.maxReoconValue);

		tmpPoint.colorR = rgb.R;

		tmpPoint.colorG = rgb.G;
		tmpPoint.colorB = rgb.B;
		tmpPoint.colorA = rgb.A;
		returnPc.push_back(tmpPoint);
	}

	return returnPc;
}

bool HUREL::Compton::RtabmapCppWrapper::LoadPlyFile(std::string filePath)
{
	return RtabmapSlamControl::instance().LoadPlyFile(filePath);
}

std::vector<double> HUREL::Compton::RtabmapCppWrapper::getMatrix3DOneLineFromPoseData()
{
	return std::vector<double>();
}

bool HUREL::Compton::RtabmapCppWrapper::GetCurrentVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStride, int* outChannelSize)
{
	static int imagesize = 0;
	
	cv::Mat color = RtabmapSlamControl::instance().GetCurrentVideoFrame();
	if (color.cols == 0)
	{		
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
		mColorImg = new uchar[imagesize];
	}
	
	memcpy(mColorImg, color.data, imagesize);
	//RtabmapCppWrapper::instance().UnlockVideoFrame();
	*outImgPtr = mColorImg;
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
	
	ReconPointCloud o3dPc = LahgiControl::instance().GetReconRealtimePointCloudCompton(reconPcDownSampled, time);

	
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


		RGBA_t rgb = ReconPointCloud::ColorScaleJet(o3dPc.reconValues_[i], 0, o3dPc.maxReoconValue);

		tmpPoint.colorR = rgb.R;

		tmpPoint.colorG = rgb.G;
		tmpPoint.colorB = rgb.B;
		tmpPoint.colorA = rgb.A;
		pc.push_back(tmpPoint);
	}

	return pc;

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