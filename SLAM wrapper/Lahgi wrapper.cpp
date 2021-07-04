#include "pch.h"

#include "Lahgi wrapper.h"

HUREL::Compton::LahgiWrapper::LahgiWrapper(eModuleManagedType type)
{
	HUREL::Compton::eMouduleType moduleType;
	switch (type)
	{
	case HUREL::Compton::eModuleManagedType::MONO:
		moduleType = HUREL::Compton::eMouduleType::MONO;
		break;
	case HUREL::Compton::eModuleManagedType::QUAD:
		moduleType = HUREL::Compton::eMouduleType::QUAD;

		break;
	case HUREL::Compton::eModuleManagedType::QUAD_DUAL:
		moduleType = HUREL::Compton::eMouduleType::QUAD_DUAL;

		break;
	default:
		break;
	}
	lahgiControlInstance.SetType(moduleType);
}

HUREL::Compton::LahgiWrapper::~LahgiWrapper()
{
	if (mReconPointCloud != NULL)
	{
		delete mReconPointCloud;
	}
}


Boolean HUREL::Compton::LahgiWrapper::AddListModeDataWraper(array<unsigned short>^ adcData, List<array<double>^>^ echks)
{
	pin_ptr<unsigned short> intParamsPtr = &adcData[0];

	unsigned short* adcS = intParamsPtr;

	std::vector<sEnergyCheck> eChkUnmanagedVector;
	eChkUnmanagedVector.resize(echks->Count);
	for each (array<double>^ e in echks)
	{
		sEnergyCheck eChkUnmanaged;
		eChkUnmanaged.minE = e[0];
		eChkUnmanaged.maxE = e[1];

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}

	lahgiControlInstance.AddListModeData(adcS, eChkUnmanagedVector);
	return true;
}

void HUREL::Compton::LahgiWrapper::GetAbsoluteListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE)
{
	std::vector<ListModeData> lists = lahgiControlInstance.GetListedListModeData();
	scatterXYZE->Clear();
	absorberXYZE->Clear();

	for (int i = 0; i < lists.size(); ++i)
	{

		array<double>^ tempScatterArray = gcnew array<double>(4);
		array<double>^ tempAbsorberArray = gcnew array<double>(4);
		tempScatterArray[0] = lists[i].Scatter.RelativeInteractionPoint[0];
		tempScatterArray[1] = lists[i].Scatter.RelativeInteractionPoint[1];
		tempScatterArray[2] = lists[i].Scatter.RelativeInteractionPoint[2];
		tempScatterArray[3] = lists[i].Scatter.InteractionEnergy;

		tempAbsorberArray[0] = lists[i].Absorber.RelativeInteractionPoint[0];
		tempAbsorberArray[1] = lists[i].Absorber.RelativeInteractionPoint[1];
		tempAbsorberArray[2] = lists[i].Absorber.RelativeInteractionPoint[2];
		tempAbsorberArray[3] = lists[i].Absorber.InteractionEnergy;


		scatterXYZE->Add(tempScatterArray);
		absorberXYZE->Add(tempAbsorberArray);		
	}
}

void HUREL::Compton::LahgiWrapper::ResetListmodeData()
{
	lahgiControlInstance.ResetListedListModeData();
	mReconPointsCount = 0;
	delete mReconPointCloud;
	mReconPointCloud = NULL;	
	for (int i = 0; i < 16; ++i)
	{
		lahgiControlInstance.ResetEnergySpectrum(i);
	}
}

void HUREL::Compton::LahgiWrapper::GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetEnergySpectrum(channelNumer).GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = static_cast<int>(eSpect.size());

	for (int i = 0; i < static_cast<int>(eSpect.size()); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = static_cast<int>(eSpect.size());

	for (int i = 0; i < static_cast<int>(eSpect.size()); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetAbsorberSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetAbsorberSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = static_cast<int>(eSpect.size());

	for (int i = 0; i < static_cast<int>(eSpect.size()); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetScatterSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = static_cast<int>(eSpect.size());

	for (int i = 0; i < static_cast<int>(eSpect.size()); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::ResetSpectrum(unsigned int channelNumber)
{
	lahgiControlInstance.ResetEnergySpectrum(channelNumber);	
}

void HUREL::Compton::LahgiWrapper::ContinueReconPointFor1m1m1m(List<array<double>^>^% vectors, List<double>^% values, double% maxValue)
{
	vectors = gcnew List< array<double>^>();
	values = gcnew List<double>();
	maxValue = 0;
	if (mReconPointCloud == NULL)
	{
		open3d::geometry::PointCloud newPC;
		newPC.points_.reserve(101 * 101 * 101);
		newPC.colors_.reserve(101 * 101 * 101);

		for (int x = -50; x <= 50; ++x)
		{
			for (int y = -50; y <= 50; ++y)
			{
				for (int z = 0; z <= 100; ++z)
				{
					Eigen::Vector3d point(x / 100, y / 100, z / 100);
					Eigen::Vector3d color(0,0,0);
					newPC.points_.push_back(point);
					newPC.colors_.push_back(color);
				}
			}
		}
		mReconPointCloud = new ReconPointCloud(newPC);
		
	}

	int count = 101 * 101 * 101;
	std::vector<ListModeData> lmData = lahgiControlInstance.GetListedListModeData();

	for (size_t i = mReconPointsCount; i < lmData.size(); ++i)
	{
		mReconPointCloud->CalculateSimpleBackProjctionCompton(lmData[i]);
	}

	mReconPointsCount = lmData.size();

	vectors->Capacity = count;
	
	maxValue = mReconPointCloud->maxReoconValue;

	for (int i = 0; i < count; ++i) {
		array<double, 1>^ poseVector = gcnew array<double>{mReconPointCloud->points_[i][0], mReconPointCloud->points_[i][1], mReconPointCloud->points_[i][2]};
		vectors->Add(poseVector);
		values->Add(mReconPointCloud->reconValues_[i]);
	}
}


void HUREL::Compton::LahgiWrapper::SaveListModeData(String^ fileName)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(fileName);
	lahgiControlInstance.SaveListedListModeData(static_cast<char*>(ptrToNativeString.ToPointer()));
}