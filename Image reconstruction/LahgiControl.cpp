#include "LahgiControl.h"
#include <future>


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

HUREL::Compton::LahgiControl::LahgiControl():
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
	mListedListModeData.resize(50000);
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

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
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
						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
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
						Eigen::Vector4d absorberPoint = mScatterModules[absorberInteractModuleNum]->FastMLPosEstimation(scatterShorts[absorberInteractModuleNum]);

						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
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
						mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
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
	return mListedListModeData;
}

void HUREL::Compton::LahgiControl::ResetListedListModeData()
{
	mListedListModeData.clear();
	mListedListModeData.resize(50000);
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
HUREL::Compton::ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloud(open3d::geometry::PointCloud& outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	time_t t = time(NULL);
	std::vector<ListModeData> tempLMData = mListedListModeData;
	std::cout << "Start Recon: " << tempLMData.size() << std::endl;
	std::cout << "Start Recon: " << reconPC.points_.size() << std::endl;

	#pragma omp parallel for
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t - tempLMData[i].InterationTime < static_cast<UINT64>(seconds))
		{
			reconPC.CalculateReconPoint(tempLMData[i], ReconPointCloud::SimpleBackprojection);
		}
		
	}
	std::cout << "End Recon: " << tempLMData.size() << std::endl;


	return reconPC;
}
