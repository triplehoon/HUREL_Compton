#include "LahgiControl.h"
#include <future>
#include <mutex>

static std::mutex mListModeDataMutex;

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
	time(&listmodeData.InterationTime);

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

void HUREL::Compton::LahgiControl::SetType(eMouduleType type)
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
	{	mScatterModules = new Module * [4];
	mAbsorberModules = new Module * [4];
	double offset = 0.083;

	double xOffset[4]{ -offset, +offset, -offset, +offset };
	double yOffset[4]{ -offset, -offset, +offset, +offset };
	for (int i = 0; i < 4; ++i)
	{
		string slutFileDirectory = string("config\\QUAD\\Scatter\\LUT\\") + to_string(i) + string(".csv");
		string sgainFileDirectory = string("config\\QUAD\\Scatter\\Gain\\") + to_string(i) + string(".csv");

		string alutFileDirectory = string("config\\QUAD\\Absorber\\LUT\\") + to_string(i) + string(".csv");
		string againFileDirectory = string("config\\QUAD\\Absorber\\Gain\\") + to_string(i) + string(".csv");


		double gain[10];
		Module::LoadGain(sgainFileDirectory, type, gain);


		double offsetZ = -(0.235);
		mScatterModules[i] = new Module(eMouduleType::QUAD, gain, gain, slutFileDirectory, xOffset[i] + T265_TO_LAHGI_OFFSET_X, yOffset[i] + T265_TO_LAHGI_OFFSET_Y, T265_TO_LAHGI_OFFSET_Z);

		Module::LoadGain(againFileDirectory, type, gain);
		mAbsorberModules[i] = new Module(eMouduleType::QUAD, gain, gain, alutFileDirectory, xOffset[i] + T265_TO_LAHGI_OFFSET_X, yOffset[i] + T265_TO_LAHGI_OFFSET_Y, offsetZ + T265_TO_LAHGI_OFFSET_Z);
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
	default:
		assert(false);
		break;
	}
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
	default:
		assert(false);
		break;
	}
	delete[] mScatterModules;
	delete[] mAbsorberModules;
}

void HUREL::Compton::LahgiControl::AddListModeDataWithTransformation(const unsigned short byteData[], std::vector<sEnergyCheck>& eChk)
{
	Eigen::Matrix4d deviceTransformation = RealsenseControl::instance().GetPoseDataEigen();



	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		const unsigned short* scatterShorts[4];
		const unsigned short* absorberShorts[4];

		for (int i = 0; i < 4; ++i)
		{
			scatterShorts[i] = &byteData[i * 9];
		}

		for (int i = 8; i < 12; ++i)
		{
			absorberShorts[i - 8] = &byteData[i * 9];
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
				mScatterModules[i]->_EnergySpectrum->AddEnergy(scattersEnergy[i]);
				mSumSpectrum.AddEnergy(scattersEnergy[i]);
				mScatterSumSpectrum.AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->_EnergySpectrum->AddEnergy(absorbersEnergy[i]);
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

		for (int i = 0; i < 4; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i][j] = byteData[i * 9 + j];
			}
		}

		for (int i = 8; i < 12; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 8][j] = byteData[i * 9 + j];
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
				mScatterModules[i]->_EnergySpectrum->AddEnergy(scattersEnergy[i]);
				mSumSpectrum.AddEnergy(scattersEnergy[i]);
				mScatterSumSpectrum.AddEnergy(scattersEnergy[i]);
				scatterInteractModuleNum = i;
				++scatterInteractionCount;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				mAbsorberModules[i]->_EnergySpectrum->AddEnergy(absorbersEnergy[i]);
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

	return mListedListModeData;
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
	// Not Yet Impletemented -> do nothing
}

EnergySpectrum HUREL::Compton::LahgiControl::GetEnergySpectrum(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return *mScatterModules[0]->_EnergySpectrum;
		}
		else if (fpgaChannelNumber == 8)
		{
			return *mAbsorberModules[0]->_EnergySpectrum;
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 4)
		{
			return *mScatterModules[fpgaChannelNumber]->_EnergySpectrum;
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 12)
		{
			return *mAbsorberModules[fpgaChannelNumber - 8]->_EnergySpectrum;
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return *mScatterModules[fpgaChannelNumber]->_EnergySpectrum;
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return *mAbsorberModules[fpgaChannelNumber - 8]->_EnergySpectrum;
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
			mScatterModules[0]->_EnergySpectrum->Reset();
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->_EnergySpectrum->Reset();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 4)
		{
			mScatterModules[fpgaChannelNumber]->_EnergySpectrum->Reset();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 12)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->_EnergySpectrum->Reset();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->_EnergySpectrum->Reset();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->_EnergySpectrum->Reset();
		}
		break;
	default:
		break;
	}
	return;
}
ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudCoded(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	time_t t = time(NULL);
	mListModeDataMutex.lock();
	std::vector<ListModeData> tempLMData = mListedListModeData;
	mListModeDataMutex.unlock();

	std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;

#pragma omp parallel for
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t - tempLMData[i].InterationTime < static_cast<__int64>(seconds))
		{
			reconPC.CalculateReconPoint(tempLMData[i], SimpleCodedBackprojection);
		}

	}
	std::cout << "End Recon: " << tempLMData.size() << std::endl;


	return reconPC;
}
HUREL::Compton::ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	time_t t = time(NULL);
	mListModeDataMutex.lock();

	std::vector<ListModeData> tempLMData = mListedListModeData;
	mListModeDataMutex.unlock();

	std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t - tempLMData[i].InterationTime < static_cast<__int64>(seconds))
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
	std::cout << "End Recon: " << tempLMData.size() << std::endl;


	return reconPC;
}

double HUREL::Compton::LahgiControl::mCalcMuOnCodedMask(double x, double y)
{
	assert(false);
	double pixelSize = 0.01;
	unsigned int indexX = UINT_MAX;
	unsigned int indexY = UINT_MAX;
	if (pow(x, 2) + pow(y, 2) < pow(0.58, 2))
	{
		//inside Circle
		if (abs(x) < 0.135 && abs(y) < 0.135)
		{
			indexX = static_cast<unsigned int>(x / pixelSize + 13.5 + 0.5);
			indexY = static_cast<unsigned int>(y / pixelSize + 13.5 + 0.5);
			assert(indexX < 37);
			assert(indexY < 37);

			return mCodeMask[indexX][indexY];
		}
		else
		{
			//But outside CM
			return 0;
		}
	}
	else
	{
		return 0;
	}

}

double HUREL::Compton::LahgiControl::mCalcIsPassOnCodedMask(double x, double y)
{
	double pixelSize = 0.01;
	unsigned int indexX = UINT_MAX;
	unsigned int indexY = UINT_MAX;
	if (pow(x, 2) + pow(y, 2) < pow(0.58, 2))
	{
		//inside Circle
		if (abs(x) < 0.185 && abs(y) < 0.185)
		{
			//inside CM
			//printf("%lf, %lf\n", x, y);

			indexX = static_cast<unsigned int>(x / pixelSize + 18 + 0.5);
			indexY = static_cast<unsigned int>(-y / pixelSize + 18 + 0.5);
			assert(indexX < 37);
			assert(indexY < 37);
			if (mCodeMask[indexX][indexY])
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			//But outside CM
			return 0;
		}
	}
	else
	{
		return 0;
	}
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

double HUREL::Compton::LahgiControl::SimpleCodedBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint)
{
	if (lmData.Type != eInterationType::CODED)
	{
		return 0;
	}
	
	Eigen::Vector4d detectorVector(0, 0, 1, 1);
	detectorVector = lmData.DetectorTransformation * detectorVector;

	double zLength = (imgPoint - lmData.Scatter.TransformedInteractionPoint.head<3>()).dot(detectorVector.head<3>());
	if (zLength < 0)
	{
		return 0;
	}

	double m = T265_To_Mask_OFFSET_Z - T265_TO_LAHGI_OFFSET_Z;
	double n = zLength - m;

	Eigen::Vector3d internalDivPoint = (m * imgPoint + n * lmData.Scatter.TransformedInteractionPoint.head<3>()) / (m + n);



	double reconValue = mCalcIsPassOnCodedMask(internalDivPoint[0] - T265_To_Mask_OFFSET_X, internalDivPoint[1] - T265_To_Mask_OFFSET_Y);

	return reconValue;
}