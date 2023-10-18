using log4net;
using Python.Runtime;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text.RegularExpressions;
using System.Windows.Threading;

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
    public class GraphData
    {
        public double X { get; set; }
        public double Y { get; set; }

        public GraphData(double x, double y)
        {
            X = x; Y = y;
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
            for (int i = 0; i < binCount + 1; ++i)
            {
                double energy = i * binSize + binSize / 2;

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

        public SpectrumEnergy(List<HistoEnergy> histoEnergies)
        {
            if (histoEnergies.Count < 10)
            {
                return;
            }
            BinSize = histoEnergies[1].Energy - histoEnergies[0].Energy;
            HistoEnergies = histoEnergies;
            for (int i = 0; i < histoEnergies.Count; ++i)
            {
                EnergyBin.Add(histoEnergies[i].Energy);
            }
            MaxEnergy = histoEnergies.Last().Energy;
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
        private static List<double> GCoeff = new List<double>();
        private static List<double> KermaCoeff = new List<double>();
        private static List<double> H10Coeff = new List<double>();
        private static bool isPyModuleLoaded = false;
        private static bool isGeDataLoaded = false;
        private void initiate()
        {
            Stopwatch sw = Stopwatch.StartNew();
            if (!isPyModuleLoaded)
            {
                if (!PythonEngine.IsInitialized)
                {
                    PythonEngine.Initialize();
                    PythonEngine.BeginAllowThreads();
                }

                using (Py.GIL())
                {
                    dynamic np = PythonEngine.ImportModule("numpy");
                    dynamic nasagamma = PythonEngine.ImportModule("nasagamma");
                }
                isPyModuleLoaded = true;

                //LogManager.GetLogger("Energy Spectrum").Info($"PyModule Loaded Elapsed: {sw.ElapsedMilliseconds} [ms]");

            }
            sw.Stop();
            if (!isGeDataLoaded)
            {
                StreamReader sr = new StreamReader("DoseData.csv");


                string? line = sr.ReadLine();
                if (line == null)
                {
                    return;
                }
                string[] data = line.Split(',');
                foreach (var s in data)
                {
                    GCoeff.Add(Convert.ToDouble(s));
                }

                line = sr.ReadLine();
                if (line == null)
                {
                    return;
                }
                data = line.Split(',');
                foreach (var s in data)
                {
                    H10Coeff.Add(Convert.ToDouble(s));
                }

                line = sr.ReadLine();
                if (line == null)
                {
                    return;
                }
                data = line.Split(',');
                foreach (var s in data)
                {
                    KermaCoeff.Add(Convert.ToDouble(s));
                }

                sr.Close();

                isGeDataLoaded = true;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>uR/hr</returns>
        public double GetExposure()
        {
            if (isGeDataLoaded && HistoEnergies.Count == 301)
            {
                double exp = 0;
                for (int i = 0; i < HistoEnergies.Count; i++)
                {
                    exp += GCoeff[i] * HistoEnergies[i].Count;
                }
                return 0.22 * exp;
            }
            else
            {
                return 0.0;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>uSv/hr//sigma</returns>
        public (double, double) GetAmbientDose(uint time)
        {
            double variance = 0;
            if (isGeDataLoaded && HistoEnergies.Count == 301)
            {
                double exp = 0;
                for (int i = 0; i < HistoEnergies.Count; i++)
                {
                    exp += H10Coeff[i] * HistoEnergies[i].Count / time;
                    //double countRateSigam = HistoEnergies[i].Count / time / time;         //231016 sbkwon : 사용사지 않은 계산 값 주석 처리
                    //variance += H10Coeff[i] * H10Coeff[i] * countRateSigam* countRateSigam;//231016 sbkwon : 사용사지 않은 계산 값 주석 처리
                }
                //return (10.5 *2 * 2.7 * exp / 5, 10.5 * 2.7 *Math.Sqrt(variance) / 5);//231016 sbkwon : 
                return (10.5 * 2 * 2.7 * exp / 5, 0);//231016 sbkwon : 10.5 * 2.7 *Math.Sqrt(variance) / 5 연산 결과는 사용하지 않아 주석처리, 추후 사용할 경우 해제 필요.
            }
            else
            {
                return (0.0, 0.0);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns>uGy/hr</returns>
        public double GetAirKerma()
        {
            if (isGeDataLoaded && HistoEnergies.Count == 301)
            {
                double exp = 0;
                for (int i = 0; i < HistoEnergies.Count; i++)
                {
                    exp += KermaCoeff[i] * HistoEnergies[i].Count;
                }
                return 0.22 * exp;
            }
            else
            {
                return 0.0;
            }
        }

        public SpectrumEnergyNasa(double binSize, double maxEnergy) : base(binSize, maxEnergy)
        {
            initiate();
        }


        public SpectrumEnergyNasa(SpectrumEnergyNasa spectrum) : base(spectrum)
        {
            initiate();
        }
        public SpectrumEnergyNasa(List<HistoEnergy> histoEnergies) : base(histoEnergies)
        {

            initiate();
        }
        public List<GraphData> SnrData = new List<GraphData>();
        public List<GraphData> PeakData = new List<GraphData>();

        private static Mutex PyMutex = new Mutex();
        public List<double> FindPeaks(float ref_x, float ref_fwhm, float fwhm_at_0, float min_snr)
        {

            List<double> PeakE = new List<double>();

            int eCount = EnergyList.Count;
            List<double> ernergyBin = new List<double>(HistoEnergies.Count);
            List<double> energyBinCount = new List<double>(HistoEnergies.Count);
            for (int i = 0; i < HistoEnergies.Count; ++i)
            {
                ernergyBin.Add(HistoEnergies[i].Energy + BinSize / 2);
                energyBinCount.Add(HistoEnergies[i].Count);
            }
            Stopwatch sw = new Stopwatch();
            sw.Start();
            PyMutex.WaitOne();
            if (!PythonEngine.IsInitialized)
            {
                PythonEngine.Initialize();
                PythonEngine.BeginAllowThreads();
            }

            IntPtr gs = PythonEngine.AcquireLock();

            using (Py.GIL())
            {
                dynamic np = PythonEngine.ImportModule("numpy");
                dynamic nasagamma = PythonEngine.ImportModule("nasagamma");
                dynamic sp = nasagamma.spectrum;
                dynamic ps = nasagamma.peaksearch;


                dynamic cts_np = np.array(energyBinCount);


                dynamic erg = np.array(ernergyBin);

                dynamic spect = sp.Spectrum(cts_np, null, erg, "keV");

                // instantiate a peaksearch object
                dynamic search = ps.PeakSearch(spect, ref_x, ref_fwhm, fwhm_at_0, min_snr);
                dynamic peakIdx = search.peaks_idx;

                dynamic snr = search.snr;

                PeakData.Clear();
                for (int i = 0; i < (int)np.size(peakIdx); ++i)
                {
                    if (peakIdx[i] < (int)np.size(erg))
                    {
                        PeakE.Add((double)erg[peakIdx[i]]);
                        PeakData.Add(new GraphData(((double)erg[peakIdx[i]]), 1000));   //sbkwon : 1000 ???? 
                    }

                }
                SnrData.Clear();
                for (int i = 0; i < (int)np.size(erg); ++i)
                {
                    SnrData.Add(new GraphData((double)erg[i], (double)snr[i]));

                }
            }
            PythonEngine.ReleaseLock(gs);

            PyMutex.ReleaseMutex();
            sw.Stop();
            //LogManager.GetLogger("Energy Spectrum").Info($"Elapsed: {sw.ElapsedMilliseconds} [ms]");




            return PeakE;
        }
    }

    //231017 sbkwon : 리스트 순서와 같게 수정
    public enum IsotopeElement
    {
        Co58,
        Co60,
        Cs137,
        Cs134,
        I131,
        Te129m,
        Ag110m,
        Pu238,
        Pu239,
        Pu240,
        Pu241,
        Ir192,
        Se75,
        U235,
        U238,
        Am241,
        Ba133,
        Na22,
        Eu152,
        Co57,
        Cd109,
        I125,
        Tc99m,
        F18,
        //K40,
        //Tl208
    }

    //231017 sbkwon :
    /// <param name="PeakEnergy">볼드체 피크 에너지</param>
    /// <param name="Priority">영상화 우선 순위</param>
    /// <param name="SubPeakEnergy">볼드체 제외 피크 에너지</param>
    public record Isotope(IsotopeElement IsotopeElement, List<double> PeakEnergy, List<int> Priority, List<double> SubPeakEnergy, string IsotopeName, string IsotopeDescription);

    public static class PeakSearching
    {
        const int _None = 9999;
        //511keV를 포함할 경우 다른 에너지 피크가 탐지되었을 경우 탐지된 것으로 간주함, 그래서 볼드체가 아니지만 볼드체로 판단
        public static readonly List<Isotope> IsotopeList = new List<Isotope>() {
            new Isotope(IsotopeElement.Co58, new List<double>(){ 810.76, 511 }, new List<int>(){_None, _None}, new List<double>(){},"Co-58", "가동중 원전"),
            new Isotope(IsotopeElement.Co60, new List<double>(){ 1173.23, 1332.49 }, new List<int>(){3, 2}, new List<double>(){ },"Co-60", "가동중 원전/비파괴 검사"),
            new Isotope(IsotopeElement.Cs137, new List<double>(){ 661.66 }, new List<int>(){ 1 }, new List<double>(){ },"Cs-137", "가동중 원전/원전 사고"),
            new Isotope(IsotopeElement.Cs134, new List<double>(){ 604.721, 795.864 }, new List<int>(){ _None, _None }, new List<double>(){ 563.246, 569.331, 1365.19 },"Cs-134", "원전 사고"),
            new Isotope(IsotopeElement.I131, new List<double>(){ 364.489 }, new List<int>(){ _None }, new List<double>(){ 80.185, 284.305, 636.989, 722.911 },"I-131", "원전 사고/의료 시설"),
            new Isotope(IsotopeElement.Te129m, new List<double>(){ 695.88 }, new List<int>(){ _None }, new List<double>(){ },"Te-129m", "원전 사고"),
            new Isotope(IsotopeElement.Ag110m, new List<double>(){ 657.76, 884.678 }, new List<int>(){ _None, _None }, new List<double>(){ 763.942, 937.485, 1384.29 },"Ag-110m", "원전 사고"),
            new Isotope(IsotopeElement.Pu238, new List<double>(){ 43.498, 152.72 }, new List<int>(){ _None, _None }, new List<double>(){ },"Pu-238 ", "원전 사고"),
            new Isotope(IsotopeElement.Pu239, new List<double>(){ 51.624, 129.296, 203.55, 332.845, 413.713, 451.481 }, new List<int>(){ _None, _None, _None, _None, _None, _None }, new List<double>(){ },"Pu-239", "원전 사고"),
            new Isotope(IsotopeElement.Pu240, new List<double>(){ 45.244 }, new List<int>(){ _None }, new List<double>(){ },"Pu-240", "원전 사고"),
            new Isotope(IsotopeElement.Pu241, new List<double>(){ 63.81, 112.75, 140.88, 148.567, 208 }, new List<int>(){ _None, _None, _None, _None, _None }, new List<double>(){ },"Pu-241", "원전 사고"),
            new Isotope(IsotopeElement.Ir192, new List<double>(){ 316.506, 468.069 }, new List<int>(){ _None, _None }, new List<double>(){ 295.957, 308.455, 604.411 },"Ir-192", "비파괴 검사"),
            new Isotope(IsotopeElement.Se75, new List<double>(){ 400.657 }, new List<int>(){ _None }, new List<double>(){ 121.116, 136, 264.658, 279.542 },"Se-75", "비파괴 검사"),
            new Isotope(IsotopeElement.U235, new List<double>(){ 143.76, 163.33, 185.715, 202.11, 205.311 }, new List<int>(){  _None, _None, _None, _None, _None }, new List<double>(){ },"U-235", "핵물질 탐지"),
            new Isotope(IsotopeElement.U238, new List<double>(){ 1001.03 }, new List<int>(){ _None }, new List<double>(){ 63.29, 92.6, 742.81, 766.36 },"U-238", "핵물질 탐지"),
            new Isotope(IsotopeElement.Am241, new List<double>(){ 59.5412 }, new List<int>(){ 4 }, new List<double>(){ },"Am-241", "체크선원"),
            new Isotope(IsotopeElement.Ba133, new List<double>(){ 356.013 }, new List<int>(){ _None }, new List<double>(){ 80.9971, 276.4, 302.851, 383.848 },"Ba-133", "체크선원"),
            new Isotope(IsotopeElement.Na22, new List<double>(){ 511, 1274.53 }, new List<int>(){ 6, 5 }, new List<double>(){ },"Na-22", "체크선원"),
            new Isotope(IsotopeElement.Eu152, new List<double>(){ 1408.01 }, new List<int>(){ _None }, new List<double>(){ 121.782, 344.279, 778.9045, 964.079, 1112.07 },"Eu-152", "체크선원"),
            new Isotope(IsotopeElement.Co57, new List<double>(){ 14.4129, 122.061, 136.474, 692.41 }, new List<int>(){ _None, _None, _None, _None }, new List<double>(){ },"Co-57", "체크선원"),
            new Isotope(IsotopeElement.Cd109, new List<double>(){ 88.0336 }, new List<int>(){ _None }, new List<double>(){ },"Cd-109", "체크선원"),
            new Isotope(IsotopeElement.I125, new List<double>(){ 35.4922 }, new List<int>(){ _None }, new List<double>(){ },"I-125", "의료 시설"),
            new Isotope(IsotopeElement.Tc99m, new List<double>(){ 140.511 }, new List<int>(){ _None }, new List<double>(){ },"Tc-99m", "의료 시설"),
            new Isotope(IsotopeElement.F18, new List<double>(){ 511 }, new List<int>(){ _None }, new List<double>(){ },"F-18", "의료 시설"),
            //new Isotope(IsotopeElement.Am241, new List<double>(){ 60 }, "Am-241", "Industrial"),
            //new Isotope(IsotopeElement.Cs137, new List<double>(){ 662 }, "Cs-137", "Industrial"),
            //new Isotope(IsotopeElement.Co60, new List<double>(){ 1173, 1332 }, "Co-60", "Industrial"),
            //new Isotope(IsotopeElement.Ba133, new List<double>(){356 }, "Ba-133", "Industrial"),
            //new Isotope(IsotopeElement.Na22, new List<double>(){511, 1275 }, "Na-22", "Industrial"),
            //new Isotope(IsotopeElement.K40, new List<double>(){1461 }, "K-40", "Background"),
            //new Isotope(IsotopeElement.Tl208, new List<double>(){2615 }, "Tl-208", "Background")
        };

        private static bool IsPeaksHasIsotope(List<double> peaks, Isotope iso, double sigma, float ref_x, float ref_fwhm, float fwhm_at_0)
        {
            foreach (double isoPeak in iso.PeakEnergy)  //모든 peak energy가 peak에 포함되어 있는지 확인
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
        //231017 sbkwon : 511keV 포함된 핵종에 대한 탐지 방법 변경
        public static List<Isotope> GetIsotopesFromPeaks(List<double> peaks, double sigma, float ref_x, float ref_fwhm, float fwhm_at_0)
        {
            List<Isotope> isotopes = new();

            bool check511 = false;      //피크에 511keV가 포함되어 있는지 여부
            bool find511iso = false;    //511keV가 포함된 핵종을 찾았는 지 여부

            //511keV 찾기
            foreach (double peak in peaks)
            {
                if (peak.Equals(511.0)) { check511 = true; break; }
            }

            foreach (Isotope iso in IsotopeList)
            {
                if (IsPeaksHasIsotope(peaks, iso, sigma, ref_x, ref_fwhm, fwhm_at_0))
                {
                    isotopes.Add(iso);

                    //511keV 포함된 핵종 찾기
                    if (check511 && (iso.IsotopeElement == IsotopeElement.Co58 || iso.IsotopeElement == IsotopeElement.Na22))
                        find511iso = true;
                }
            }

            if (check511)
            {
                Isotope? iso = isotopes.Find(x => x.IsotopeElement == IsotopeElement.F18);

                if (find511iso && iso != null)
                    isotopes.Remove(iso);
                //F18은 511keV 가 존재하면 Isotopes에 추가 되기 때문에 코드 추가 하지 않음.
            }

            return isotopes;
        }
        public static double CalcFWHM(double x, double ref_x, double ref_fwhm, double fwhm_at_0)
        {
            double f0 = fwhm_at_0;
            double f1 = ref_fwhm;
            double x1 = ref_x;

            return (f1 / Math.Sqrt(x1) * Math.Sqrt(x)) + f0;
        }
    }
}
