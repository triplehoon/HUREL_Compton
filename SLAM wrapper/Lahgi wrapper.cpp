#include "pch.h"

#include "Lahgi wrapper.h"

using namespace HUREL::Compton;

HUREL::Compton::LahgiWrapper::LahgiWrapper()
{
}

bool HUREL::Compton::LahgiWrapper::Initiate(eModuleManagedType type)
{
	WrapperLogger::Log::Info("C++CLR::HUREL::Compton::LahgiWrapper", "Initiate");
	HUREL::Compton::eModuleCppWrapper moduleType = HUREL::Compton::eModuleCppWrapper::MONO;
	switch (type)
	{
	case HUREL::Compton::eModuleManagedType::MONO:
		moduleType = HUREL::Compton::eModuleCppWrapper::MONO;
		break;
	case HUREL::Compton::eModuleManagedType::QUAD:
		moduleType = HUREL::Compton::eModuleCppWrapper::QUAD;

		break;
	case HUREL::Compton::eModuleManagedType::QUAD_DUAL:
		moduleType = HUREL::Compton::eModuleCppWrapper::QUAD_DUAL;
		break;
	default:
		break;
	}
	return LahgiCppWrapper::instance().SetType(moduleType);
}

void HUREL::Compton::LahgiWrapper::AddListModeDataWraper(array<unsigned short>^ adcData)
{
	pin_ptr<unsigned short> intParamsPtr = &adcData[0];

	LahgiCppWrapper::instance().AddListModeDataWithTransformation(intParamsPtr);
	
}

void HUREL::Compton::LahgiWrapper::SetEchks(List<array<double>^>^ echks)
{

	std::vector<std::vector<double>> eChkUnmanagedVector;
	eChkUnmanagedVector.reserve(echks->Count);
	for each (array<double> ^ e in echks)
	{
		std::vector<double> eChkUnmanaged;
		eChkUnmanaged.reserve(2);
		double minE = e[0];
		double maxE = e[1];
		eChkUnmanaged.push_back(minE);
		eChkUnmanaged.push_back(maxE);

		eChkUnmanagedVector.push_back(eChkUnmanaged);
	}

	LahgiCppWrapper::instance().SetEchks(eChkUnmanagedVector);
}

void HUREL::Compton::LahgiWrapper::GetRelativeListModeData(List<array<double>^>^% scatterXYZE, List<array<double>^>^% absorberXYZE)
{
	std::vector<ListModeDataCppWrapper> lists = LahgiCppWrapper::instance().GetRelativeListModeData();
	scatterXYZE->Clear();
	absorberXYZE->Clear();

	for (int i = 0; i < lists.size(); ++i)
	{
		array<double>^ tempScatterArray = gcnew array<double>(4);
		array<double>^ tempAbsorberArray = gcnew array<double>(4);
		tempScatterArray[0] = lists[i].ScatterRelativeInteractionPointX;
		tempScatterArray[1] = lists[i].ScatterRelativeInteractionPointY;
		tempScatterArray[2] = lists[i].ScatterRelativeInteractionPointZ;
		tempScatterArray[3] = lists[i].ScatterInteractionEnergy;

		tempAbsorberArray[0] = lists[i].AbsorberRelativeInteractionPointX;
		tempAbsorberArray[1] = lists[i].AbsorberRelativeInteractionPointY;
		tempAbsorberArray[2] = lists[i].AbsorberRelativeInteractionPointZ;
		tempAbsorberArray[3] = lists[i].AbsorberInteractionEnergy;


		scatterXYZE->Add(tempScatterArray);
		absorberXYZE->Add(tempAbsorberArray);		
	}
}

void HUREL::Compton::LahgiWrapper::ResetListmodeData()
{
	WrapperLogger::Log::Info("C++CLR::UREL::Compton::LahgiWrapper", "Reset list mode data");
	LahgiCppWrapper::instance().ResetListedListModeData();	
}

void HUREL::Compton::LahgiWrapper::GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSpectrum(channelNumer);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetEcal(unsigned int channelNumer, double% ecalA, double% ecalB, double% ecalC)
{
	std::tuple<double, double, double> ecals = LahgiCppWrapper::instance().GetEcalValue(channelNumer);

	ecalA = std::get<0>(ecals);
	ecalB = std::get<1>(ecals);
	ecalC = std::get<2>(ecals);
}

void HUREL::Compton::LahgiWrapper::SetEcal(unsigned int channelNumer, double ecalA, double ecalB, double ecalC)
{
	std::tuple<double, double, double> ecals = std::make_tuple(static_cast<double>(ecalA), static_cast<double>(ecalB), static_cast<double>(ecalC));
	LahgiCppWrapper::instance().SetEcalValue(channelNumer, ecals);
}

void HUREL::Compton::LahgiWrapper::GetSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetSumSpectrum();

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
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetAbsorberSumSpectrum();

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
	LahgiCppWrapper::instance().SaveListedListModeData(static_cast<char*>(ptrToNativeString.ToPointer()));
}

bool HUREL::Compton::LahgiWrapper::LoadListModeData(System::String^ filePath)
{
	IntPtr ptrToNativeString = Marshal::StringToHGlobalAnsi(filePath);
	return LahgiCppWrapper::instance().LoadListedListModeData(static_cast<char*>(ptrToNativeString.ToPointer()));
}

void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetScatterSumSpectrum();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetScatterSumSpectrum(time);

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, static_cast<double>(eSpect[i].Count)};
		energyCount->Add(tempECount);
	}

}

void HUREL::Compton::LahgiWrapper::GetAbsorberSumSpectrumByTime(List<array<double>^>^% energyCount, unsigned int time)
{
	std::vector<BinningEnergy> eSpect = LahgiCppWrapper::instance().GetAbsorberSumSpectrum(time);

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
	LahgiCppWrapper::instance().RestEnergySpectrum(channelNumber);
}

void HUREL::Compton::LahgiWrapper::GetRealTimeReconImage(double time, eReconType reconType, int% width, int% height, int% stride, IntPtr% data)
{
	//cv::Mat color = cv::Mat();
	//switch (reconType)
	//{
	//case HUREL::Compton::eReconType::CODED:
	//{
	//	color = lahgiControlInstance->GetListModeImageCoded(time);
	//	break;
	//}
	//case HUREL::Compton::eReconType::COMPTON:
	//	color = lahgiControlInstance->GetListModeImageCompton(time);
	//	break;
	//case HUREL::Compton::eReconType::HYBRID:
	//	break;
	//default:
	//	color = lahgiControlInstance->GetListModeImageCompton(time);
	//	break;
	//}


	//if (color.cols > 1) {
	//	width = color.cols;
	//	height = color.rows;
	//	stride = color.step;
	//	void* ptr = static_cast<void*>(color.ptr());

	//	if (ptr == NULL)
	//	{
	//		return;
	//	}

	//	data = IntPtr(ptr);
	//}
}

void HUREL::Compton::LahgiWrapper::Logging(std::string className, std::string msg)
{
	System::String^ classNameManage = gcnew System::String(className.c_str());
	System::String^ msgManage = gcnew System::String(msg.c_str());
	WrapperLogger::Log::Info(classNameManage, msgManage);
}

