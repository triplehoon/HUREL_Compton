#include "LahgiControl.h"
#include <future>
#include <mutex>
#include <open3d/visualization/utility/DrawGeometry.h>

static std::mutex mListModeDataMutex;
static std::mutex mListModeImageMutex;

using namespace std;
using namespace HUREL;
using namespace Compton;


ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation, std::chrono::milliseconds timeInMili)
{
	InteractionData scatter;
	InteractionData absorber;
	ListModeData listmodeData;

	if (!isnan(scatterEnergy))
	{
		scatter.InteractionEnergy = scatterEnergy;
	}
	scatter.RelativeInteractionPoint = scatterPoint;
	scatter.TransformedInteractionPoint = transformation * scatterPoint;

	if (!isnan(absorberEnergy))
	{
		absorber.InteractionEnergy = absorberEnergy;
	}
	if (iType == eInterationType::COMPTON)
	{
		
		absorber.RelativeInteractionPoint = absorberPoint;
		absorber.TransformedInteractionPoint = transformation * absorberPoint;
	}	



	listmodeData.Type = iType;
	listmodeData.Scatter = scatter;
	listmodeData.Absorber = absorber;
	listmodeData.DetectorTransformation = transformation;
	listmodeData.InteractionTimeInMili = timeInMili;
	
	return listmodeData;
}

ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType& iType, Eigen::Vector4d& scatterPoint, Eigen::Vector4d& absorberPoint, double& scatterEnergy, double& absorberEnergy, Eigen::Matrix4d& transformation)
{
	InteractionData scatter;
	InteractionData absorber;
	ListModeData listmodeData;

	if (!isnan(scatterEnergy))
	{
		scatter.InteractionEnergy = scatterEnergy;
	}
	scatter.RelativeInteractionPoint = scatterPoint;
	scatter.TransformedInteractionPoint = transformation * scatterPoint;

	if (!isnan(absorberEnergy))
	{
		absorber.InteractionEnergy = absorberEnergy;
	}
	if (iType == eInterationType::COMPTON)
	{

		absorber.RelativeInteractionPoint = absorberPoint;
		absorber.TransformedInteractionPoint = transformation * absorberPoint;
	}	



	listmodeData.Type = iType;
	listmodeData.Scatter = scatter;
	listmodeData.Absorber = absorber;
	listmodeData.DetectorTransformation = transformation;
	listmodeData.InteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());;

	return listmodeData;
}


HUREL::Compton::LahgiControl::LahgiControl() :
	mAbsorberModules(NULL),
	mScatterModules(NULL),
	mModuleType(HUREL::Compton::eMouduleType::MONO)
{	
	Eigen::Matrix4d test;
	test = Matrix4d::Ones();
	Eigen::Vector3d test2 = Eigen::Vector3d(1,1,1);
	
	Eigen::Vector4d test3;
	test3 = Eigen::Vector4d(1, 2, 3, 4);
	test3.normalize();

	
	t265toLACCPosTransform << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Logger loaded in cpp!", eLoggerType::INFO);
	//this->LoadListedListModeData("20220706_DigitalLabScan_100uCi_-1,0,2.4_cpplmdata.csv");
}

HUREL::Compton::LahgiControl& HUREL::Compton::LahgiControl::instance()
{
	static LahgiControl& instance = *(new LahgiControl());
	return instance;
}

bool HUREL::Compton::LahgiControl::SetType(eMouduleType type)
{
	HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::LahgiControl", "Set type", eLoggerType::INFO);
	mListModeDataMutex.lock();

	mListedListModeData.reserve(50000);	
	mListModeDataMutex.unlock();
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
	std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	Eigen::Matrix4d deviceTransformation = RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransform;

	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		Eigen::Array<float, 1, 9> scatterShorts[4];
		Eigen::Array<float, 1, 9> absorberShorts[4];
		
		//Channel 4 to 7
		for (int i = 4; i < 8; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i - 4][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		//Channel 12 to 16
		for (int i = 12; i < 16; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 12][j] = static_cast<double>(byteData[i * 9 + j]);
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
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i], timeInMili);
				//mSumSpectrum.AddEnergy(scattersEnergy[i]);
				//mScatterSumSpectrum.AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i], timeInMili);
				//mSumSpectrum.AddEnergy(absorbersEnergy[i]);
				//mAbsorberSumSpectrum.AddEnergy(absorbersEnergy[i]);
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
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::COMPTON;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimation(absorberShorts[absorberInteractModuleNum]);

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili));
					}					
				}
			}
			else if (absorberInteractionCount == 0)
			{
				//Coded Apature
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy> eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);

						if (IsOnActiveArea(scatterPoint[0], scatterPoint[1], *mScatterModules[scatterInteractModuleNum]))
						{
							mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
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

void HUREL::Compton::LahgiControl::AddListModeDataWithTransformationVerification(const unsigned short byteData[], std::vector<sEnergyCheck>& eChk)
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
			scatterShorts[i - 4] = &byteData[i * 9];
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
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
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
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::COMPTON;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimationVerification(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimationVerification(absorberShorts[absorberInteractModuleNum]);

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
					}
				}
			}
			else if (absorberInteractionCount == 0)
			{
				//Coded Apature
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy> eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimationVerification(scatterShorts[scatterInteractModuleNum]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);

						if (IsOnActiveArea(scatterPoint[0], scatterPoint[1], *mScatterModules[scatterInteractModuleNum]))
						{
							mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
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
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
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
void HUREL::Compton::LahgiControl::AddListModeDataEigen(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation, std::vector<sEnergyCheck> eChk)
{
	
}

HUREL::Compton::eMouduleType HUREL::Compton::LahgiControl::GetDetectorType()
{
	return mModuleType;
}

const std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData() const
{
	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	lmData.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}


	return lmData;
}

std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData()
{
	size_t size = mListedListModeData.size();
	std::vector<ListModeData> lmData;
	lmData.reserve(size);
	for (int i = 0; i < size; ++i)
	{
		lmData.push_back(mListedListModeData[i]);
	}
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

bool HUREL::Compton::LahgiControl::LoadListedListModeData(std::string fileName)
{
	std::ifstream loadFile;
	loadFile.open(fileName);
	if (!loadFile.is_open())
	{
		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Fail to open file", eLoggerType::ERROR_t);
		loadFile.close();
		return false;
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
	if (mListedListModeData.size() == 0)
	{

		HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Fail to load lm data", eLoggerType::ERROR_t);
		return false;
	}

	HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "Load lm data: " + fileName, eLoggerType::INFO);
	return true;
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
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber - 4]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 12]->GetEnergySpectrum();
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
	EnergySpectrum spect;
	switch (mModuleType)
	{
	case eMouduleType::MONO:

		spect =	mScatterModules[0]->GetEnergySpectrum() + mAbsorberModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	default:
		break;
	}
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetAbsorberSumEnergySpectrum()
{
	EnergySpectrum spect;
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		spect = mAbsorberModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			
			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{
			
			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	default:
		break;
	}
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetScatterSumEnergySpectrum()
{
	EnergySpectrum spect;
	switch (mModuleType)
	{
	case eMouduleType::MONO:

		spect = mScatterModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
			
		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
			
		}
		break;
	default:
		break;
	}
	return spect;
}

std::tuple<double, double, double> HUREL::Compton::LahgiControl::GetEcalValue(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return mScatterModules[0]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber == 8)
		{
			return mAbsorberModules[0]->GetEnergyCalibration();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber - 4]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 12]->GetEnergyCalibration();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergyCalibration();
		}
		break;
	default:
		break;
	}
	return tuple<double, double, double>();
}

void HUREL::Compton::LahgiControl::SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			mScatterModules[0]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber - 4]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 12]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->SetEnergyCalibration(get<0>(ecal), get<1>(ecal), get<2>(ecal));
		}
		break;
	default:
		break;
	}
}

void HUREL::Compton::LahgiControl::ResetEnergySpectrum(int fpgaChannelNumber)
{	
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
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber - 4]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 12]->GetEnergySpectrum().Reset();
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
		vector<ListModeData> tmp = GetListedListModeData();
		if (tmp.size() < startIdx || tmp.size() == 0)
		{

			startIdx = 0;
			continue;
		}
		
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

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if ((t - tempLMData[i].InteractionTimeInMili).count() < seconds * 1000)
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

	std::vector<ListModeData> tempLMData = GetListedListModeData();

	//std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	//std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (seconds == 0)
		{
			reconStartIndex = 0;
			break;
		}
		if (t.count() - tempLMData[i].InteractionTimeInMili.count() < static_cast<__int64>(seconds))
		{
			reconStartIndex = i;
			break;
		}

	}

	std::vector<ListModeData> reconLm;
	reconLm.reserve(tempLMData.size());
	//assert(0, "temp energy search");
	for (const auto lm : tempLMData)
	{
		if (lm.Absorber.InteractionEnergy + lm.Scatter.InteractionEnergy > 620 && lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 700)
		{
			reconLm.push_back(lm);
		}
		
	}

#pragma omp parallel for
	for (int i = 0; i < reconLm.size(); ++i)
	{
		reconPC.CalculateReconPoint(reconLm[i], ReconPointCloud::SimpleComptonBackprojection);
	}
	//HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "GetReconRealtimePointCloudCompton End Recon: " + reconLm.size(), eLoggerType::INFO);


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
