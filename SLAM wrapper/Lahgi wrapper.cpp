#include "pch.h"

#include "Lahgi wrapper.h"

HUREL::Compton::LahgiWrapper::LahgiWrapper()
{
	
}

bool HUREL::Compton::LahgiWrapper::Initiate(eModuleManagedType type)
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
	return lahgiControlInstance.SetType(moduleType);
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

	lahgiControlInstance.AddListModeDataWithTransformation(adcS, eChkUnmanagedVector);
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
	for (int i = 0; i < 16; ++i)
	{
		lahgiControlInstance.ResetEnergySpectrum(i);
	}
}

void HUREL::Compton::LahgiWrapper::GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetEnergySpectrum(channelNumer).GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetAbsorberSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetAbsorberSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::SaveListModeData(System::String^ fileName)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(fileName);
	lahgiControlInstance.SaveListedListModeData(static_cast<char*>(ptrToNativeString.ToPointer()));
}


void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetScatterSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::ResetSpectrum(unsigned int channelNumber)
{
	lahgiControlInstance.ResetEnergySpectrum(channelNumber);	
}

void HUREL::Compton::LahgiWrapper::GetRealTimeReconImage(double time, eReconType reconType, int% width, int% height, int% stride, IntPtr% data)
{
	cv::Mat color = cv::Mat();
	switch (reconType)
	{
	case HUREL::Compton::eReconType::CODED:
	{
		color = lahgiControlInstance.GetListModeImageCoded(time);
		break;
	}
	case HUREL::Compton::eReconType::COMPTON:
		color = lahgiControlInstance.GetListModeImageCompton(time);
		break;
	case HUREL::Compton::eReconType::HYBRID:
		break;
	default:
		color = lahgiControlInstance.GetListModeImageCompton(time);
		break;
	}


	if (color.cols > 1) {
		width = color.cols;
		height = color.rows;
		stride = color.step;
		void* ptr = static_cast<void*>(color.ptr());

		if (ptr == NULL)
		{
			return;
		}

		data = IntPtr(ptr);
	}
}
