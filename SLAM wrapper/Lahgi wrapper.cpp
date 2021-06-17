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
}

void HUREL::Compton::LahgiWrapper::GetSpectrum(unsigned int channelNumer, List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetEnergySpectrum(channelNumer).GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, eSpect[i].Count};
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
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, eSpect[i].Count};
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
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, eSpect[i].Count};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::GetScatterSumSpectrum(List<array<double>^>^% energyCount)
{
	std::vector<BinningEnergy> eSpect = lahgiControlInstance.GetScatterSumEnergySpectrum().GetHistogramEnergy();

	energyCount = gcnew List<array<double>^>();
	energyCount->Capacity = eSpect.size();

	for (int i = 0; i < eSpect.size(); ++i)
	{
		array<double, 1>^ tempECount = gcnew array<double>{eSpect[i].Energy, eSpect[i].Count};
		energyCount->Add(tempECount);
	}
}

void HUREL::Compton::LahgiWrapper::ResetSpectrum(unsigned int channelNumber)
{
	lahgiControlInstance.ResetEnergySpectrum(channelNumber);	
}

void HUREL::Compton::LahgiWrapper::GetSLAMReconPointCloud(List<array<double>^>^% vectors, List<array<double>^>^% colors)
{
	throw gcnew System::NotImplementedException();
}

void HUREL::Compton::LahgiWrapper::GetRealTimeReconImage(double time, List<array<double>^>^% colorAlphas, List<array<float>^>^% uvs)
{
	colorAlphas = gcnew List< array<double>^>();
	uvs = gcnew List< array<float>^>();
	
//	realsenseControlInstance = RealsenseControl::instance();

	int size;
	
	if (!realsenseControlInstance.IsPipeLineOn) {
		return;
	}

	auto rtPC =  realsenseControlInstance.GetRTPointCloudTransposed();
	open3d::geometry::PointCloud pose = std::get<0>(rtPC);
	std::vector<Eigen::Vector2f> uv = std::get<1>(rtPC);


	ReconPointCloud rcPC = lahgiControlInstance.GetReconRealtimePointCloud(pose, time);


	if (pose.IsEmpty()) {
		return;
	}

	if (pose.colors_.size() > pose.points_.size())
	{
		size = pose.points_.size();
	}
	else
	{
		size = pose.colors_.size();
	}
	if (uv.size() < size) {
		size = uv.size();
	}

	if (uv.size() == 0)
	{
		return;
	}

	double maxValue = -DBL_MAX;
	for (unsigned int i = 0; i < rcPC.points_.size(); ++i)
	{
		if (maxValue < rcPC.reconValues_[i])
		{
			maxValue = rcPC.reconValues_[i];
		}
	}

	colorAlphas->Capacity = size;
	uvs->Capacity = size;
	
	for (int i = 0; i < size; i++) 
	{
		RGBA_t color =  ReconPointCloud::ColorScaleJet(rcPC.reconValues_[i], maxValue * 0.8, maxValue);

		array<double, 1>^ colorAlphaVector = gcnew array<double>{color.R, color.G, color.B, color.A};
		colorAlphas->Add(colorAlphaVector);

		array<float, 1>^ uvVector = gcnew array<float>{uv[i][0], uv[i][1]};
		uvs->Add(uvVector);
	}
}
