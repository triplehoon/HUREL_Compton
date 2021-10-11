using MathNet.Numerics;
using MathNet.Numerics.Distributions;
using Python.Runtime;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;

namespace HUREL.Compton.RadioisotopeAnalysis
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

        protected List<double> EnergyBin = new List<double>();
        public List<double> EnergyList = new List<double>();

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
            EnergyList.Add(energy);
            for (int i = 0; i < EnergyBin.Count - 1; ++i)
            {
                if (energy > EnergyBin[i] && energy < EnergyBin[i + 1])
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
            EnergyList.Clear();
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
       
        public void SaveSpectrumData(string path, string fileName)
        {
            List<HistoEnergy> tmpHistoE = new List<HistoEnergy>(HistoEnergies.Count);
            for (int i = 0; i < HistoEnergies.Count; ++i)
            {
                tmpHistoE.Add(HistoEnergies[i]);
            }
            int tmpListCount = EnergyList.Count;
            List<double> tmpListData = new List<double>(tmpListCount);
            for (int i = 0; i < tmpListCount; ++i)
            {
                tmpListData.Add(EnergyList[i]);
            }

            string csvPath = Path.Combine(path.ToString(), DateTime.Now.ToString("yyyyMMddHHmm") + "_" + fileName + "_Spectrum.csv");
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(csvPath))
            {
                //file.WriteLine("Time[HHMMssFFF],SCposX[m],SCposY,SCposZ,SCEnergy[keV],ABposX,ABposY,ABposZ,ABEnergy");
                file.WriteLine("Energy,Count");
                foreach (HistoEnergy hist in tmpHistoE)
                {
                    file.WriteLine($"{hist.Energy},{hist.Count}");
                }
            }
            string csvListPath = Path.Combine(path.ToString(), DateTime.Now.ToString("yyyyMMddHHmm") + "_" + fileName + "_ListEnergyData.csv");
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(csvListPath))
            {
                //file.WriteLine("Time[HHMMssFFF],SCposX[m],SCposY,SCposZ,SCEnergy[keV],ABposX,ABposY,ABposZ,ABEnergy");
                file.WriteLine("Energy[keV]");
                foreach (double e in tmpListData)
                {
                    file.WriteLine($"{e}");
                }
            }
        }

        public bool IsEmpty()
        {
            foreach (HistoEnergy hist in HistoEnergies)
            {
                if (hist.Count > 0)
                {
                    return false;
                }
            }
            return true;
        }

        public bool LoadSpectrumData(string path)
        {
            using(StreamReader file = new StreamReader(path))
            {
                string firstLine = file.ReadLine();
                if (firstLine != "Energy,Count")
                {
                    return false;
                }
                else
                {
                    HistoEnergies.Clear();
                    while (!file.EndOfStream)
                    {
                        string line = file.ReadLine();
                        string[] value = line.Split(',');
                        HistoEnergies.Add(new HistoEnergy(Convert.ToDouble(value[0]), Convert.ToInt32(value[1])));
                    }
                }
            }
            return true;
        }
        public bool LoadEnergyListModeData(string path)
        {        
            using (StreamReader file = new StreamReader(path))
            {
                string firstLine = file.ReadLine();
                if (firstLine != "Energy[keV]")
                {
                    return false;
                }
                else
                {
                    this.Reset();
                    while (!file.EndOfStream)
                    {
                        string line = file.ReadLine();
                        AddEnergy(Convert.ToDouble(line));
                    }
                }
            }
            return true;
        }
    }

    public class SpectrumEnergyNasa : SpectrumEnergy
    {
        public SpectrumEnergyNasa(double binSize, double maxEnergy) : base(binSize, maxEnergy)
        {
            try
            {
                using (Py.GIL())
                {
                    dynamic np = Py.Import("numpy");
                    dynamic nasagamma = Py.Import("nasagamma");
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("EnergyCalibration: No Python module error");
                Console.WriteLine(e.Message);
                Debug.Assert(false, "Spectrum NASA not available");
            }
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
        public SpectrumEnergyNasa(SpectrumEnergyNasa spectrum) : base(spectrum)
        {
            BinSize = spectrum.BinSize;
            MaxEnergy = spectrum.MaxEnergy;
            EnergyBin = spectrum.EnergyBin;
            HistoEnergies = spectrum.HistoEnergies;
        }

        public List<double> FindPeaks(float ref_x, float ref_fwhm, float fwhm_at_0, float min_snr)
        {

            List<double> PeakE = new List<double>();

            int eCount = EnergyList.Count;
            List<double> listData = new List<double>(EnergyList.Count); 
            
            for (int i = 0; i < eCount; ++i)
            {
                listData.Add(EnergyList[i]);
            }
            using (Py.GIL())
            {           
                dynamic np = Py.Import("numpy");
                dynamic nasagamma = Py.Import("nasagamma");
                dynamic sp = nasagamma.spectrum;
                dynamic ps = nasagamma.peaksearch;

                dynamic eData = np.array(listData);
                dynamic bin = np.arange(0, 3000, 5);
                dynamic hist = np.histogram(eData, bin);
                dynamic cts_np = hist[0];
                bin = hist[1];
                dynamic erg = np.resize(bin, (np.size(bin) - 1));

                dynamic spect = sp.Spectrum(cts_np, null, erg, "keV");

                // instantiate a peaksearch object
                dynamic search = ps.PeakSearch(spect, ref_x, ref_fwhm, fwhm_at_0, min_snr);
                dynamic peakIdx = search.peaks_idx;

                for (int i = 0; i < (int)np.size(peakIdx); ++i)
                {
                    PeakE.Add((double)erg[peakIdx[i]]);
                }
            }

            return PeakE;
        }
    }

    public enum IsotopeElement
    {
        Am241,
        Cs137,
        Co60,
        Na22,
        Ba133
    }
    public record Isotope(IsotopeElement IsotopeElement, List<double> PeakEnergy, string IsotopeName);

    public static class PeakSearching
    {
        public static readonly List<Isotope> IsotopeList = new List<Isotope>() { 
            new Isotope(IsotopeElement.Am241, new List<double>(){ 60 }, "Am-241"),
            new Isotope(IsotopeElement.Cs137, new List<double>(){ 662 }, "Cs-137"),
            new Isotope(IsotopeElement.Co60, new List<double>(){ 1173, 1332 }, "Co-60"),
            new Isotope(IsotopeElement.Ba133, new List<double>(){ 276, 356 }, "Ba-133"),
            new Isotope(IsotopeElement.Na22, new List<double>(){511, 1275 }, "Na-22"),
        };

        private static bool IsPeaksHasIsotope(List<double> peaks, Isotope iso, double sigma, float ref_x, float ref_fwhm, float fwhm_at_0)
        {
            foreach (double isoPeak in iso.PeakEnergy)
            {
                double fwhm = CalcFWHM(isoPeak, ref_x, ref_fwhm, fwhm_at_0);
                bool hasFindPeak = false;
                foreach (double peak in peaks)
                {
                    if (isoPeak - sigma * fwhm < peak && isoPeak + sigma * fwhm > peak)
                    {
                        hasFindPeak = true;
                    }
                }

                if (!hasFindPeak)
                {
                    return false;
                }
            }
            return true;
        }
        public static List<Isotope> GetIsotopesFromPeaks(List<double> peaks, double sigma, float ref_x, float ref_fwhm, float fwhm_at_0)
        {
            List<Isotope> isotopes = new();

            foreach (Isotope iso in IsotopeList)
            {
                if (IsPeaksHasIsotope(peaks, iso, sigma, ref_x, ref_fwhm, fwhm_at_0))
                {
                    isotopes.Add(iso);
                }
            }
          
            return isotopes;
        }
        private static double CalcFWHM(double x, double ref_x, double ref_fwhm, double fwhm_at_0)
        {
            double f0 = fwhm_at_0;
            double f1 = ref_fwhm;
            double x1 = ref_x;

            return (f1 / Math.Sqrt(x1) * Math.Sqrt(x)) + f0;
        }
    }
}
