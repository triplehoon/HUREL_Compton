#include "LahgiControl.h"
#include <future>
#include <mutex>

static std::mutex mListModeDataMutex;
static std::mutex mListModeImageMutex;

using namespace std;
using namespace HUREL;
using namespace Compton;


ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation)
{
	InteractionData scatter;
	InteractionData absorber;
	ListModeData listmodeData;

	scatter.InteractionEnergy = scatterEnergy;
	scatter.RelativeInteractionPoint = scatterPoint;
	scatter.TransformedInteractionPoint = transformation * scatterPoint;


	if (iType == eInterationType::COMPTON)
	{
		absorber.InteractionEnergy = absorberEnergy;
		absorber.RelativeInteractionPoint = absorberPoint;
		absorber.TransformedInteractionPoint = transformation * absorberPoint;
	}


	listmodeData.Type = iType;
	listmodeData.Scatter = scatter;
	listmodeData.Absorber = absorber;
	listmodeData.DetectorTransformation = transformation;
	time(&listmodeData.InteractionTime);
	listmodeData.InteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	
	return listmodeData;
}

HUREL::Compton::LahgiControl::LahgiControl() :
	mAbsorberModules(NULL),
	mScatterModules(NULL),
	mModuleType(HUREL::Compton::eMouduleType::MONO)
{	
}

HUREL::Compton::LahgiControl& HUREL::Compton::LahgiControl::instance()
{
	static LahgiControl* instance = new LahgiControl();
	return *instance;
}

bool HUREL::Compton::LahgiControl::SetType(eMouduleType type)
{
	mListModeDataMutex.lock();

	mListedListModeData.reserve(50000);	
	mListModeDataMutex.unlock();

	mSumSpectrum = EnergySpectrum(5, 3000);
	mScatterSumSpectrum = EnergySpectrum(5, 3000);
	mAbsorberSumSpectrum = EnergySpectrum(5, 3000);
	mModuleType = type;
	switch (type)
	{
	case HUREL::Compton::eMouduleType::MONO:
		assert(false);
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{	
		mScatterModules = new Module * [4];
	mAbsorberModules = new Module * [4];
	double offset = 0.083;
	// 3 0
	// 2 1
	//
	double xOffset[4]{ -offset, -offset, +offset, +offset };
	double yOffset[4]{ +offset, -offset, -offset, +offset };
	string scatterSerial = "50777";
	string absorberSerial = "50784";
	for (int i = 0; i < 4; ++i)
	{
		double gain[10];
		
		
		double offsetZ = -(0.220);
		mScatterModules[i] = new Module(eMouduleType::QUAD, "config\\QUAD", scatterSerial + string("_Scint") + to_string(i), xOffset[i], yOffset[i] , 0);		
		if (!mScatterModules[i]->IsModuleSet())
		{
			return false;
		}
		mAbsorberModules[i] = new Module(eMouduleType::QUAD, "config\\QUAD", absorberSerial + string("_Scint") + to_string(i), xOffset[i], yOffset[i], offsetZ);
		if (!mAbsorberModules[i]->IsModuleSet())
		{
			return false;
		}
	}
	break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		//double offset = 0.083;

		//double xOffset[8]{ -offset, +offset, -offset, +offset };
		//double yOffset[8]{ -offset, -offset, +offset, +offset };
		//mScatterModules = new Module * [8];
		//mAbsorberModules = new Module * [8];
		//for (int i = 0; i < 8; ++i)
		//{
		//	string slutFileDirectory = string("config\\QUAD\\Scatter\\LUT\\Lut_scintillator_") + to_string(i) + string(".csv");
		//	string sgainFileDirectory = string("config\\QUAD\\Scatter\\Gain\\Energy_gain_scintillator_") + to_string(i) + string(".csv");

		//	string alutFileDirectory = string("config\\QUAD\\Absorber\\LUT\\Lut_scintillator_") + to_string(i) + string(".csv");
		//	string againFileDirectory = string("config\\QUAD\\Absorber\\Gain\\Energy_gain_scintillator_") + to_string(i) + string(".csv");


		//	double gain[9];
		//	Module::LoadGain(sgainFileDirectory, type, gain);


		//	double offsetZ = -(0.251 + (31.5 - 21.5) / 1000);
		//	mScatterModules[i] = new Module(eMouduleType::QUAD, gain, gain, slutFileDirectory, xOffset[i], yOffset[i]);

		//	Module::LoadGain(againFileDirectory, type, gain);
		//	mAbsorberModules[i] = new Module(eMouduleType::QUAD, gain, gain, alutFileDirectory, xOffset[i], yOffset[i]);

		//}
		break;
	}
	case HUREL::Compton::eMouduleType::TEST:
	{
		break;
	}
	default:
		assert(false);
		break;
	}

	return true;
}

HUREL::Compton::LahgiControl::~LahgiControl()
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		assert(false);
		break;
	case HUREL::Compton::eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			delete mScatterModules[i];
			delete mAbsorberModules[i];
		}
		break;
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
		/*mScatterModules = new Module * [8];
		mAbsorberModules = new Module * [8];
		for (int i = 0; i < 8; ++i)
		{
			delete mScatterModules[i];
			delete mAbsorberModules[i];
		}*/
		break;
	case HUREL::Compton::eMouduleType::TEST:
		break;
	default:
		assert(false);
		break;
	}
	delete[] mScatterModules;
	delete[] mAbsorberModules;
}

void HUREL::Compton::LahgiControl::AddListModeDataWithTransformation(const unsigned short byteData[], std::vector<sEnergyCheck>& eChk)
{
	Eigen::Matrix4d t265toLACCPosTransform;
	t265toLACCPosTransform << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		   					0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		   					0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		   					0, 0, 0, 1;
	Eigen::Matrix4d deviceTransformation = RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransform;

	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		const unsigned short* scatterShorts[4];
		const unsigned short* absorberShorts[4];

		for (int i = 4; i < 8; ++i)
		{
			scatterShorts[i-4] = &byteData[i * 9];
		}

		for (int i = 12; i < 16; ++i)
		{
			absorberShorts[i - 12] = &byteData[i * 9];
		}


		double scattersEnergy[4];
		double absorbersEnergy[4];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum = 4;
		int absorberInteractModuleNum = 4;

		for (int i = 0; i < 4; ++i)
		{
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);
				mSumSpectrum.AddEnergy(scattersEnergy[i]);
				mScatterSumSpectrum.AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
				mSumSpectrum.AddEnergy(absorbersEnergy[i]);
				mAbsorberSumSpectrum.AddEnergy(absorbersEnergy[i]);
				absorberInteractModuleNum = i;
				++absorberInteractionCount;
			}
		}

		if (scatterInteractionCount == 1)
		{
			double sEnergy = scattersEnergy[scatterInteractModuleNum];
			double aEnergy = absorbersEnergy[absorberInteractModuleNum];

			if (absorberInteractionCount == 1)
			{
				//Compton
				eInterationType type = eInterationType::COMPTON;
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimation(absorberShorts[absorberInteractModuleNum]);
						mListModeDataMutex.lock();

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
						mListModeDataMutex.unlock();

					}
				}
			}
			else if (absorberInteractionCount == 0)
			{
				//Coded Apature
				eInterationType type = eInterationType::CODED;
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy> eChk[i].minE)
					{
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);

						if (IsOnActiveArea(scatterPoint[0], scatterPoint[1], *mScatterModules[scatterInteractModuleNum]))
						{
							mListModeDataMutex.lock();
							mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
							mListModeDataMutex.unlock();
						}

					}
				}

			}
		}
		else
		{
			eInterationType type = eInterationType::NONE;
		}



		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		//Do nothing
		break;
	}
	}
}

void HUREL::Compton::LahgiControl::AddListModeData(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation, std::vector<sEnergyCheck> eChk)
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		unsigned short scatterShorts[4][9];
		unsigned short absorberShorts[4][9];
		//Channel 4 to 8
		for (int i = 4; i < 8; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i-4][j] = byteData[i * 9 + j];
			}
		}

		//Channel 12 to 16
		for (int i = 12; i < 16; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 12][j] = byteData[i * 9 + j];
			}
		}


		double scattersEnergy[4];
		double absorbersEnergy[4];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum = 4;
		int absorberInteractModuleNum = 4;

		for (int i = 0; i < 4; ++i)
		{
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);
				mSumSpectrum.AddEnergy(scattersEnergy[i]);
				mScatterSumSpectrum.AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
				mSumSpectrum.AddEnergy(absorbersEnergy[i]);
				mAbsorberSumSpectrum.AddEnergy(absorbersEnergy[i]);
				absorberInteractModuleNum = i;
				++absorberInteractionCount;
			}
		}

		if (scatterInteractionCount == 1)
		{
			double sEnergy = scattersEnergy[scatterInteractModuleNum];
			double aEnergy = absorbersEnergy[absorberInteractModuleNum];

			if (absorberInteractionCount == 1)
			{
				//Compton
				eInterationType type = eInterationType::COMPTON;
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimation(scatterShorts[absorberInteractModuleNum]);
						mListModeDataMutex.lock();

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
						mListModeDataMutex.unlock();

					}
				}
			}
			else if (absorberInteractionCount == 0)
			{
				//Coded Apature
				eInterationType type = eInterationType::CODED;
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
						mListModeDataMutex.lock();

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
						mListModeDataMutex.unlock();

					}
				}

			}
		}
		else
		{
			eInterationType type = eInterationType::NONE;
		}



		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		//Do nothing
		break;
	}
	}
}

HUREL::Compton::eMouduleType HUREL::Compton::LahgiControl::GetDetectorType()
{
	return mModuleType;
}

const std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData() const
{
	mListModeDataMutex.lock();
	std::vector<ListModeData> lmData = mListedListModeData;
	mListModeDataMutex.unlock();

	return lmData;
}

std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData()
{
	mListModeDataMutex.lock();
	std::vector<ListModeData> lmData = mListedListModeData;
	mListModeDataMutex.unlock();

	return lmData;
}

void HUREL::Compton::LahgiControl::ResetListedListModeData()
{
	mListModeDataMutex.lock();

	mListedListModeData.clear();
	mListedListModeData.reserve(50000);
	mListModeDataMutex.unlock();

}

void HUREL::Compton::LahgiControl::SaveListedListModeData(std::string fileName)
{
	std::ofstream saveFile;
	saveFile.open(fileName);
	if (!saveFile.is_open()) 
	{
		std::cout << "File is not opened" << endl;
		saveFile.close();
		return;
	}
	std::vector<ListModeData> data = this->GetListedListModeData();
	for (unsigned int i = 0; i < data.size(); ++i)
	{
		ListModeData& d = data[i];
		saveFile << d.WriteListModeData() << std::endl;
	}
	saveFile.close();


	return;
}

void HUREL::Compton::LahgiControl::LoadListedListModeData(std::string fileName)
{
	std::ifstream loadFile;
	loadFile.open(fileName);
	if (!loadFile.is_open())
	{
		std::cout << "File is not opened" << endl;
		loadFile.close();
		return;
	}
	mListedListModeData.clear();
	string buffer;
	char line[2048];
	while (loadFile.good())
	{
		ListModeData temp;
		getline(loadFile, buffer);
		if (temp.ReadListModeData(buffer))
		{
			mListedListModeData.push_back(temp);
		}
	}

	loadFile.close();
	return;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetEnergySpectrum(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return mScatterModules[0]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber == 8)
		{
			return mAbsorberModules[0]->GetEnergySpectrum();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 4)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 12)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum();
		}
		break;
	default:
		break;
	}
	return EnergySpectrum();
}

EnergySpectrum HUREL::Compton::LahgiControl::GetSumEnergySpectrum()
{
	return mSumSpectrum;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetAbsorberSumEnergySpectrum()
{
	return mAbsorberSumSpectrum;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetScatterSumEnergySpectrum()
{
	return mScatterSumSpectrum;
}

void HUREL::Compton::LahgiControl::ResetEnergySpectrum(int fpgaChannelNumber)
{
	mSumSpectrum.Reset();
	mScatterSumSpectrum.Reset();
	mAbsorberSumSpectrum.Reset();
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			mScatterModules[0]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->GetEnergySpectrum().Reset();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 4)
		{
			mScatterModules[fpgaChannelNumber]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 12)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum().Reset();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum().Reset();
		}
		break;
	default:
		break;
	}
	return;
}

static std::future<void> t1;

void HUREL::Compton::LahgiControl::StartListModeGenPipe(double miliSec)
{
	if (mIsListModeGenOn == true)
	{
		StopListModeGenPipe();
	}
	printf("ListMode Gen Pipe starts\n");
	mListModeImgInterval = miliSec;
	mIsListModeGenOn = true;
	mListModeImage.clear();
	auto func = std::bind(&LahgiControl::ListModeGenPipe, this);
	t1 = std::async(std::launch::async, func);
}

void HUREL::Compton::LahgiControl::StopListModeGenPipe()
{
	if (mIsListModeGenOn == false)
	{
		return;
	}
	mIsListModeGenOn = false;
	t1.get();

	printf("ListMode Gen Pipe is end\n");
}

void HUREL::Compton::LahgiControl::ResetListModeImage()
{
	mListModeImage.clear();
}

void HUREL::Compton::LahgiControl::ListModeGenPipe()
{
	size_t startIdx = 0;
	while (mIsListModeGenOn)
	{
		Sleep(mListModeImgInterval);
		mListModeDataMutex.lock();
		vector<ListModeData> tmp = mListedListModeData;
		if (tmp.size() < startIdx || tmp.size() == 0)
		{
			mListModeDataMutex.unlock();

			startIdx = 0;
			continue;
		}
		mListModeDataMutex.unlock();
		
		long long timeInMiliStart = tmp[startIdx].InteractionTimeInMili.count();

		for (size_t i = startIdx; i < tmp.size() - 1; ++i)
		{
			long long timeInMiliEnd = tmp[i + 1].InteractionTimeInMili.count();

			if (timeInMiliEnd - timeInMiliStart > mListModeImgInterval)
			{
				vector<ListModeData>::const_iterator first = tmp.begin() + startIdx;
				vector<ListModeData>::const_iterator last = tmp.begin() + i + 2;
				vector<ListModeData> effectiveData(first, last);
				RadiationImage effectiveImg(effectiveData);

				mListModeImageMutex.lock();
				mListModeImage.push_back(effectiveImg);
				mListModeImageMutex.unlock();
				startIdx = i;
				timeInMiliStart = tmp[startIdx].InteractionTimeInMili.count();
			}
		}
	}
	
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudComptonUntransformed(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	time_t t = time(NULL);
	mListModeDataMutex.lock();

	std::vector<ListModeData> tempLMData = mListedListModeData;
	mListModeDataMutex.unlock();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t - tempLMData[i].InteractionTime < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

#pragma omp parallel for
	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPoint(tempLMData[i], ReconPointCloud::SimpleComptonBackprojectionUntransformed);
	}

	std::cout << "End Recon: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeDataMutex.lock();

	std::vector<ListModeData> tempLMData = mListedListModeData;
	mListModeDataMutex.unlock();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

#pragma omp parallel for
	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPoint(tempLMData[i], ReconPointCloud::SimpleComptonBackprojection);
	}
	std::cout << "GetReconRealtimePointCloudCompton End Recon: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudCoded(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeImageMutex.lock();

	std::vector<RadiationImage> tempLMData = mListModeImage;
	mListModeImageMutex.unlock();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointCoded(tempLMData[i]);
	}
	std::cout << "End GetReconOverlayPointCloudCoded: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudCompton(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeImageMutex.lock();

	std::vector<RadiationImage> tempLMData = mListModeImage;
	mListModeImageMutex.unlock();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointCompton(tempLMData[i]);
	}
	//std::cout << "End GetReconOverlayPointCloudCompton: " << tempLMData.size() << std::endl;


	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudHybrid(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeImageMutex.lock();

	std::vector<RadiationImage> tempLMData = mListModeImage;
	mListModeImageMutex.unlock();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointHybrid(tempLMData[i]);
	}
	//std::cout << "End GetReconOverlayPointCloudHybrid: " << tempLMData.size() << std::endl;


	return reconPC;
}

cv::Mat HUREL::Compton::LahgiControl::GetListModeImageCoded(double time)
{
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	
	mListModeImageMutex.lock();
	std::vector<RadiationImage> tempLMData = mListModeImage;
	mListModeImageMutex.unlock();

	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(time))
		{
			reconStartIndex = i;
			break;
		}

	}
	cv::Mat img = tempLMData[reconStartIndex].mCodedImage;
	for (int i = reconStartIndex + 1; i < tempLMData.size(); ++i)
	{
		img + tempLMData[i].mCodedImage;
	}
	return img;
}

cv::Mat HUREL::Compton::LahgiControl::GetListModeImageCompton(double time)
{
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	
	mListModeImageMutex.lock();
	std::vector<RadiationImage> tempLMData = mListModeImage;
	mListModeImageMutex.unlock();

	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(time))
		{
			reconStartIndex = i;
			break;
		}

	}
	cv::Mat img = tempLMData[reconStartIndex].mComptonImage;
	for (int i = reconStartIndex + 1; i < tempLMData.size(); ++i)
	{
		img + tempLMData[i].mComptonImage;
	}
	return img;
}

cv::Mat HUREL::Compton::LahgiControl::GetListModeImageHybrid(double time)
{
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	
	mListModeImageMutex.lock();
	std::vector<RadiationImage> tempLMData = mListModeImage;
	mListModeImageMutex.unlock();

	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<__int64>(time))
		{
			reconStartIndex = i;
			break;
		}

	}
	cv::Mat img = tempLMData[reconStartIndex].mHybridImage;
	for (int i = reconStartIndex + 1; i < tempLMData.size(); ++i)
	{
		img + tempLMData[i].mHybridImage;
	}
	return img;
}

bool HUREL::Compton::LahgiControl::IsOnActiveArea(double x, double y, Module& module)
{
	double realPosX = x - module.mModuleOffsetX;
	double realPosY = y - module.mModuleOffsetY;

	if ((abs(realPosX) < ACTIVE_AREA_LENGTH / 2) && (abs(realPosY) < ACTIVE_AREA_LENGTH / 2))
	{
		return true;
	}
	else
	{
		return false;
	}
}
