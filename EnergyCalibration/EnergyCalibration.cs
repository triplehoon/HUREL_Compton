using MathNet.Numerics;
using MathNet.Numerics.Distributions;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

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

        public double BinSize = 0;
        public double MaxEnergy = 0;
        public SpectrumEnergy(double binSize, double maxEnergy)
        {
            BinSize = binSize;
            MaxEnergy = maxEnergy;
            int binCount = (int)(MaxEnergy / binSize);
            for (int i = 0; i < binCount; ++i)
            {
                double energy = i * binSize;

                EnergyBin.Add(energy);
                HistoEnergies.Add(new HistoEnergy(energy));
            }
        }
        public SpectrumEnergy(SpectrumEnergy spectrum)
        {
            BinSize = spectrum.BinSize;
            MaxEnergy = spectrum.MaxEnergy;
            EnergyBin = spectrum.EnergyBin;
            HistoEnergies = spectrum.HistoEnergies;
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

        public List<double> FindPeaks(double diffLimit = -20)
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
            double dataMin = 500000;
            //Find min
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
                        Debug.WriteLine($"Spectrum diff min: {data}, E: {HistoEnergies[i - 1].Energy}");
                    }
                }
                if (data < dataMin)
                {
                    dataMin = data;
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
                    isotopes.Add(new Isotope(ISOTOPE.Cs137, 662 ));
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
                        isotopes.Add(new Isotope(ISOTOPE.Co60, 1173));
                        isotopes.Add(new Isotope(ISOTOPE.Co60, 1332));
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
                        isotopes.Add(new Isotope(ISOTOPE.Na22, 1275));
                        isotopes.Add(new Isotope(ISOTOPE.Na22, 511));
                    }
                }

                //Am241
                //if (IsEnergyInPeakEnergy(energy, 60))
                //{
                //    isotopes.Add(new Isotope(ISOTOPE.Am241, new List<double> { 59.5 }));
                //}

                //Ba133
                if (IsEnergyInPeakEnergy(energy, 276))
                {
                    bBa133Find[0] = true;
                    
                }
                if (IsEnergyInPeakEnergy(energy, 356))
                {
                    if (bBa133Find[0])
                    {
                        isotopes.Add(new Isotope(ISOTOPE.Ba133, 356 ));
                        isotopes.Add(new Isotope(ISOTOPE.Ba133, 276 ));                        
                    }
                }
            }
            return isotopes;
        }        
        private static bool IsEnergyInPeakEnergy(double energy, double peak)
        {
            if (peak < energy + 100 && peak > energy - 100)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        public void SaveSpectrumData(string path, string fileName)
        {

            string csvPath = Path.Combine(path.ToString(), DateTime.Now.ToString("yyyyMMddHHmm") + "_" + fileName + "_Spectrum.csv");
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(csvPath))
            {
                //file.WriteLine("Time[HHMMssFFF],SCposX[m],SCposY,SCposZ,SCEnergy[keV],ABposX,ABposY,ABposZ,ABEnergy");
                file.WriteLine("Energy,Count");
                foreach(HistoEnergy hist in HistoEnergies)
                {
                    file.WriteLine($"{hist.Energy},{hist.Count}");
                }
            }
        }

        public bool IsEmpty()
        {
            foreach(HistoEnergy hist in HistoEnergies)
            {
                if (hist.Count > 0)
                {
                    return true;
                }
            }
            return false;
        }

        public bool LoadSpectrumData(string path)
        {
            using (StreamReader file = new StreamReader(path))
            {
                string firstLine = file.ReadLine();
                if (firstLine != "Energy,Count")
                {
                    return false;
                }
                else
                {
                    HistoEnergies.Clear();
                    while(!file.EndOfStream)
                    {
                        string line = file.ReadLine();
                        string[] value = line.Split(',');
                        HistoEnergies.Add(new HistoEnergy(Convert.ToDouble(value[0]), Convert.ToInt32(value[1])));
                    }
                }
            }
            return true;
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

    public record Isotope(ISOTOPE IsotopeName, double PeakEnergy);

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
            return FWHM;
        }
    }
}
