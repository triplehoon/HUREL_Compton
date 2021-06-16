using System;
using System.Collections.Generic;

namespace HUREL.Compton
{

    public class HistoEnergy
    {
        public double Energy { get; set; }
        public int Count { get; set; }
        public HistoEnergy(double energy)
        {
            Energy = energy;

        }

        public HistoEnergy(double energy, int count)
        {
            Energy = energy;
            Count = count;

        }

    }
    public class SpectrumEnergy
    {
        public List<HistoEnergy> HistoEnergies = new List<HistoEnergy>();

        private List<double> EnergyBin = new List<double>();

        public SpectrumEnergy(int binSize, double MaxEnergy)
        {
            int binCount = (int)(MaxEnergy / binSize);
            for (int i = 0; i < binCount; ++i)
            {
                double energy = i * binSize;

                EnergyBin.Add(energy);
                HistoEnergies.Add(new HistoEnergy(energy));
            }
        }

        public void AddEnergy(double energy)
        {
            for (int i = 0; i < EnergyBin.Count - 1; ++i)
            {
                if (energy < EnergyBin[i + 1] && energy > EnergyBin[i])
                {
                    ++HistoEnergies[i].Count;
                    break;
                }
            }
        }
        public void AddEnergy(List<double> energy)
        {
            foreach (double d in energy)
            {
                AddEnergy(d);
            }
        }

        public void Reset()
        {
            foreach (var data in HistoEnergies)
            {
                data.Count = 0;
            }
        }
    }
    public class EnergyCalibration
    {
        static double GetFWHM(List<HistoEnergy> histoEnergies, double minE, double maxE)
        {
            return 0;
        }
    }
}
