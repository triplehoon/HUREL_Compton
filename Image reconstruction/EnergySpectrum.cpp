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



void HUREL::Compton::EnergySpectrum::AddEnergy(double energy)
{
    if (energy >= mMaxEnergy || energy < 0)
    {
        return;
    }
	++mHistogramEnergy[floor(energy / mBinSize)].Count;
}

void HUREL::Compton::EnergySpectrum::Reset()
{
    for (unsigned int i = 0; i < mHistogramEnergy.size(); ++i)
    {
        mHistogramEnergy[i].Count = 0;
    }
}  


EnergySpectrum HUREL::Compton::EnergySpectrum::operator+(const EnergySpectrum& rhs) const
{
    if (this->mBinSize != rhs.mBinSize || this->mMaxEnergy != rhs.mMaxEnergy)
    {
        return EnergySpectrum();
    }
    EnergySpectrum value = EnergySpectrum(this->mBinSize, this->mMaxEnergy);
    for (unsigned int i = 0; i < value.mHistogramEnergy.size(); ++i)
    {
        value.mHistogramEnergy[i].Count = this->mHistogramEnergy[i].Count + rhs.mHistogramEnergy[i].Count;
    }

    return value;
}

