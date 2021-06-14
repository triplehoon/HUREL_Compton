#include "pch.h"

#include "Lahgi wrapper.h"

HUREL::Compton::LahgiWrapper::LahgiWrapper(eModuleManagedType type)
{
	HUREL::Compton::eMouduleType moduleType = HUREL::Compton::eMouduleType::MONO;
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

bool HUREL::Compton::LahgiWrapper::AddListModeData(array<UINT16>^ adcData, List<array<double>^>^ echks)
{
	unsigned short adcS[144];
	for (int i = 0; i < 144; ++i)
	{
		adcS[i] = adcData[i];
	}

	std::vector<sEnergyCheck> eChkUnmanagedVector;
	
	for each (array<double>^ e in echks)
	{
		sEnergyCheck eChkUnmanaged;
		eChkUnmanaged.minE = e[0];
		eChkUnmanaged.maxE = e[1];

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}

	//lahgiControlInstance.AddListModeData(adcS, realsenseControlInstance.GetPoseDataEigen(), eChkUnmanagedVector);
	return true;
}

void HUREL::Compton::LahgiWrapper::GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE)
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
}

void HUREL::Compton::LahgiWrapper::GetSLAMReconPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	throw gcnew System::NotImplementedException();
}

void HUREL::Compton::LahgiWrapper::GetRealTimeReconPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors, List<array<float>^>^% uvs)
{
	throw gcnew System::NotImplementedException();
}
