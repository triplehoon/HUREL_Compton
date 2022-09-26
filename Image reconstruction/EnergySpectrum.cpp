#include "EnergySpectrum.h"
#include <cassert>

using namespace HUREL::Compton;


HUREL::Compton::EnergySpectrum::EnergySpectrum(unsigned int binSize, double maxEnergy)
{
    assert(binSize > 0);
    int binCount = static_cast<unsigned int>(maxEnergy / binSize);
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
   
    for (unsigned long long i = 0; i < mHistogramEnergy.size() - 1; ++i)
    {
        if (energy < mEnergyBin[i + 1] && energy > mEnergyBin[i])
        {
            ++mHistogramEnergy[i].Count;
            break;
        }
    }
    EnergyTime etime;
    etime.Energy = energy;
    etime.InteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
}

void HUREL::Compton::EnergySpectrum::Reset()
{
    for (unsigned int i = 0; i < mHistogramEnergy.size(); ++i)
    {
        mHistogramEnergy[i].Count = 0;
    }
    mEnergyList.clear();
}  
