#include "EnergySpectrum.h"
#include <cassert>
#include <mutex>

using namespace HUREL::Compton;

std::mutex resetMutex;

HUREL::Compton::EnergySpectrum::EnergySpectrum(unsigned int binSize, double maxEnergy)
{
    assert(binSize > 0);
    int binCount = static_cast<unsigned int>(maxEnergy / binSize);
    mBinSize = static_cast<double>(binSize);
    mMaxEnergy = maxEnergy;
    for (unsigned long long i = 0; i < binCount + 1; ++i)
    {
        double energy = static_cast<double>(i * binSize + binSize / 2);

        mEnergyBin.push_back(energy);
        mHistogramEnergy.push_back(BinningEnergy{energy, 0});
    }

}


std::vector<BinningEnergy> HUREL::Compton::EnergySpectrum::GetHistogramEnergy()
{
    return mHistogramEnergy;
}

std::vector<EnergyTime> HUREL::Compton::EnergySpectrum::GetEnergyList()
{
    size_t size = mEnergyList.size();
    std::vector<EnergyTime> returnValue;
    if (size == 0)
    {
        return returnValue;
    }
    returnValue.reserve(size);
    for (int i = 0; i < size; ++i)
    {
        returnValue.push_back(mEnergyList[i]);
    }
    return returnValue;
}


void HUREL::Compton::EnergySpectrum::AddEnergy(double energy)
{
    if (energy >= mMaxEnergy || energy < 0)
    {
        return;
    }
	++mHistogramEnergy[floor(energy / mBinSize)].Count;

    EnergyTime etime;
    etime.Energy = energy;
    etime.InteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    ResetEnergyListMutex.lock();
    mEnergyList.push_back(etime);
    ResetEnergyListMutex.unlock();

}

void HUREL::Compton::EnergySpectrum::AddEnergy(double energy, std::chrono::milliseconds& timeInMili)
{
    if (energy >= mMaxEnergy || energy < 0)
    {
        return;
    }
    ++mHistogramEnergy[floor(energy / mBinSize)].Count;

    EnergyTime etime;
    etime.Energy = energy;
    etime.InteractionTimeInMili = timeInMili;
    ResetEnergyListMutex.lock();
    mEnergyList.push_back(etime);
    ResetEnergyListMutex.unlock();
}

void HUREL::Compton::EnergySpectrum::Reset()
{
    for (unsigned int i = 0; i < mHistogramEnergy.size(); ++i)
    {
        mHistogramEnergy[i].Count = 0;
    }
    ResetEnergyListMutex.lock();
    mEnergyList.clear();
    mEnergyList.shrink_to_fit();
    ResetEnergyListMutex.unlock();
}  

EnergySpectrum HUREL::Compton::EnergySpectrum::operator+(EnergySpectrum rhs)
{
    EnergySpectrum result(rhs.mBinSize, rhs.mMaxEnergy);
    std::vector<EnergyTime> eListRhs = rhs.GetEnergyList();
    std::vector<EnergyTime> eListThis = this->GetEnergyList();
    
    for (int i = 0; i < eListRhs.size(); ++i)
    {
        result.AddEnergy(eListRhs[i].Energy, eListRhs[i].InteractionTimeInMili);
    } 
    for (int i = 0; i < eListThis.size(); ++i)
    {
        result.AddEnergy(eListThis[i].Energy, eListThis[i].InteractionTimeInMili);
    }
    
    return result;    
}

EnergySpectrum HUREL::Compton::EnergySpectrum::operator=(EnergySpectrum rhs)
{
    this->mEnergyList = rhs.mEnergyList;
    this->mHistogramEnergy = rhs.mHistogramEnergy;
    this->mEnergyBin = rhs.mEnergyBin;
    this->mBinSize = rhs.mBinSize;
    this->mMaxEnergy = rhs.mMaxEnergy;
    return *this;
}

HUREL::Compton::EnergySpectrum::EnergySpectrum(const EnergySpectrum& copy)
{
    this->mEnergyList = copy.mEnergyList;
    this->mHistogramEnergy = copy.mHistogramEnergy;
    this->mEnergyBin = copy.mEnergyBin;
    this->mBinSize = copy.mBinSize;
    this->mMaxEnergy = copy.mMaxEnergy;
}

void HUREL::Compton::EnergySpectrum::SaveEnergySpectrum(std::string filePath)
{
    std::ofstream saveFile;

    
    
    saveFile.open(filePath);
    if (!saveFile.is_open())
    {
        saveFile.close();
        return;
    }
    std::vector<EnergyTime> data = this->GetEnergyList();

    for (unsigned int i = 0; i < data.size(); ++i)
    {
        EnergyTime& d = data[i];
        saveFile << d.InteractionTimeInMili.count() << "," << d.Energy << "\n";
    }
    saveFile.close();
}
