using MathNet.Numerics;
using MathNet.Numerics.Distributions;
using System;
using System.Collections.Generic;
using System.Diagnostics;


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

    }
    public class SpectrumEnergy
    {
        public List<HistoEnergy> HistoEnergies = new List<HistoEnergy>();

        private List<double> EnergyBin = new List<double>();

        public double BinSize = 0;

        public SpectrumEnergy(double binSize, double MaxEnergy)
        {
            BinSize = binSize;
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
            foreach (HistoEnergy data in HistoEnergies)
            {
                data.Count = 0;
            }
        }

        public List<double> FindPeaks()
        {
            //derevate energys
            List<double> numericDiff = new List<double>();
            List<double> numericSecondDiff = new List<double>();


            List<double> peaks = new List<double>();

            numericDiff.Capacity = EnergyBin.Count;
            numericSecondDiff.Capacity = EnergyBin.Count;

            numericDiff.Add(HistoEnergies[1].Count - HistoEnergies[0].Count);
            numericDiff.Add(HistoEnergies[2].Count - HistoEnergies[1].Count);
            numericSecondDiff.Add(numericDiff[1] - numericDiff[0]);
            //Find min
            double diffLimit = -20;
            bool flagIsDecending = true;
            for (int i = 1; i < EnergyBin.Count - 2; ++i)
            {
                numericDiff.Add(HistoEnergies[i + 1].Count - HistoEnergies[i].Count);
                numericSecondDiff.Add(numericDiff[i] - numericDiff[i - 1]);

                double data = numericSecondDiff[i];
                double dataFoward = numericSecondDiff[i - 1];
                if (data < diffLimit)
                {
                    if (flagIsDecending && (data - dataFoward) > 0)
                    {
                        peaks.Add(HistoEnergies[i - 1].Energy);
                    }
                }

                flagIsDecending = dataFoward - data > 0;
            }

            return peaks;
        }
        static public List<Isotope> GetIsotopesFromPeaks(List<double> peaks)
        {
            List<Isotope> isotopes = new();

#if DEBUG
            double tempPeak = 0;
            foreach (double peak in peaks)
            {
                Debug.Assert(peak > tempPeak);
                tempPeak = peak;
            }
#endif
            bool[] bCo60Find = new bool[2] { false, false };
            bool[] bNa22Find = new bool[2] { false, false };
            bool[] bBa133Find = new bool[3] { false, false, false };

            foreach (var energy in peaks)
            {
                //Cs137
                if (IsEnergyInPeakEnergy(energy, 662))
                {
                    isotopes.Add(new Isotope(ISOTOPE.Cs137, new List<double> { 662 }));
                }

                //Co60
                if (IsEnergyInPeakEnergy(energy, 1173))
                {
                    bCo60Find[0] = true;
                }
                if (IsEnergyInPeakEnergy(energy, 1332))
                {
                    if (bCo60Find[0])
                    {
                        isotopes.Add(new Isotope(ISOTOPE.Co60, new List<double> { 1173, 1332 }));
                    }
                }

                //Na22
                if (IsEnergyInPeakEnergy(energy, 511))
                {
                    bNa22Find[0] = true;    
                }
                if (IsEnergyInPeakEnergy(energy, 1275))
                {
                    if (bNa22Find[0])
                    {
                        isotopes.Add(new Isotope(ISOTOPE.Na22, new List<double> { 511, 1275 }));
                    }
                }

                //Am241
                if (IsEnergyInPeakEnergy(energy, 60))
                {
                    isotopes.Add(new Isotope(ISOTOPE.Am241, new List<double> { 59.5 }));
                }

                //Ba133
                if (IsEnergyInPeakEnergy(energy, 81))
                {
                    bBa133Find[0] = true;
                }
                if (IsEnergyInPeakEnergy(energy, 276))
                {
                    bBa133Find[1] = true;
                    
                }
                if (IsEnergyInPeakEnergy(energy, 356))
                {
                    if (bBa133Find[0] && bBa133Find[1])
                    {
                        isotopes.Add(new Isotope(ISOTOPE.Ba133, new List<double> { 81, 276, 356 }));
                    }
                }

            }

            return isotopes;
        }
        private static bool IsEnergyInPeakEnergy(double energy, double peak)
        {
            if (peak < energy + 50 && peak > energy - 50)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    public enum ISOTOPE
    {
        Am241,
        Cs137,
        Co60,
        Na22,
        Ba133
    }

    public record Isotope(ISOTOPE IsotopeName, List<double> Peaks);

    public class EnergyCalibration
    {
        public static double GetFWHM(List<HistoEnergy> histoEnergies,  int minEIdx, int maxEIdx)
        {
            //fitting gaussian
            List<HistoEnergy> range = histoEnergies.GetRange(minEIdx, maxEIdx - minEIdx);

            var gaussian = new Func<double, double, double, double>((σ, μ, x) =>
            {
                return Normal.PDF(μ, σ, x);
            });


            double[] x = new double[range.Count];
            double[] y = new double[range.Count];   
            for (int i = 0; i < range.Count; i++)
            {
                x[i] = range[i].Energy;
                y[i] = range[i].Count;
            }


            var fit = Fit.Curve(x, y, gaussian, 1, range[(maxEIdx - minEIdx) / 2].Energy);

            var FWHM = fit.Item1 * 2.35;
            return 0;
        }
    }
}
