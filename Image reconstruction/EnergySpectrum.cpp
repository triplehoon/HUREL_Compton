#include "EnergySpectrum.h"


using namespace HUREL::Compton;


HUREL::Compton::EnergySpectrum::EnergySpectrum(unsigned int binSize, double maxEnergy)
{
    int binCount = static_cast<int>(maxEnergy / binSize);
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

HUREL::Compton::EnergySpectrum::EnergySpectrum(int binSize, double maxEnergy)
{
}

HUREL::Compton::EnergySpectrum::EnergySpectrum()
{
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
