#include "EnergySpectrum.h"
#include <cassert>

using namespace HUREL::Compton;


HUREL::Compton::EnergySpectrum::EnergySpectrum(unsigned int binSize, double maxEnergy):
mMaxEnergy(maxEnergy), mBinSize(binSize)
{
    assert(binSize > 0);
    int binCount = static_cast<unsigned int>(maxEnergy / binSize);
    for (unsigned long long i = 0; i < binCount; ++i)
    {
        double energy = static_cast<double>(i * binSize);

        mEnergyBin.push_back(energy);
        mHistogramEnergy.push_back(BinningEnergy{energy, 0});
    }
}

std::vector<BinningEnergy> HUREL::Compton::EnergySpectrum::GetHistogramEnergy()
{
    return mHistogramEnergy;
}

std::vector<double> HUREL::Compton::EnergySpectrum::CalcPeakEnergys()
{
    //differentails

    size_t size = mHistogramEnergy.size();
    std::vector<double> diffVector;
    diffVector.reserve(size);
    for (size_t i = 0; i < size; ++i)
    {

    }


    return std::vector<double>();
}


void HUREL::Compton::EnergySpectrum::AddEnergy(double energy)
{
    for (unsigned long long i = 0; i < mHistogramEnergy.size() - 1; ++i)
    {
        if (energy < mEnergyBin[i + 1] && energy > mEnergyBin[i])
        {
            ++mHistogramEnergy[i].Count;
            break;
        }
    }
}

void HUREL::Compton::EnergySpectrum::Reset()
{
    for (unsigned int i = 0; i < mHistogramEnergy.size(); ++i)
    {
        mHistogramEnergy[i].Count = 0;
    }
}  
