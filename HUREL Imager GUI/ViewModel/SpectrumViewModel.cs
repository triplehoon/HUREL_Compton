using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    public class IsotopeInfo
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public string Energy { get; set; }

        public IsotopeInfo(string name, string description, string energy)
        {
            Name = name;
            Description = description;
            Energy = energy;
        }
    }
    public enum eSpectrumCases
    {
        Scatter,
        Absorber,
        All,
        ByChannel
    };

    public class SpectrumViewModel : ViewModelBase
    {

        public SpectrumViewModel()
        {
            LahgiApi.StatusUpdate += StatusUpdate;
            EnergySpectrum = new ObservableCollection<HistoEnergy>();
            IsotopeInfos = new ObservableCollection<IsotopeInfo>();
            Random random = new Random();
            for (int i = 0; i < 3000; i++)
            {
                EnergySpectrum.Add(new HistoEnergy(i, random.Next(5000)));
            }
        }
        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }

            SpectrumEnergyNasa? spectrum = null;
            switch (_spectrumCases)
            {
                case eSpectrumCases.Scatter:
                    spectrum = LahgiApi.GetScatterSumSpectrum();
                    break;
                case eSpectrumCases.Absorber:
                    spectrum = LahgiApi.GetAbsorberSumSpectrum();
                    break;
                case eSpectrumCases.All:
                    spectrum = LahgiApi.GetSumSpectrumEnergy();
                    break;
                case eSpectrumCases.ByChannel:
                    spectrum = LahgiApi.GetSpectrumEnergy(FpgaChannelNumber);
                    break;
            }
            if (spectrum != null)
            {
                EnergySpectrum = new ObservableCollection<HistoEnergy>(spectrum.HistoEnergies);
                if (eventArgs is LahgiApiEnvetArgs)
                {
                    LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                    if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                    {
                        EnergySpectrum.Clear();
                        var espect = spectrum;

                        foreach (var e in espect.HistoEnergies)
                        {
                            EnergySpectrum.Add(e);
                        }
                        ObservableCollection<IsotopeInfo> isotopeInfos = new ObservableCollection<IsotopeInfo>();
                        List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(espect.FindPeaks(662, 50, 10, 2), 1, 662, 40, 5);

                        foreach (Isotope iso in DetectedIso)
                        {
                            string energy = "";
                            foreach (double e in iso.PeakEnergy)
                            {
                                energy += e.ToString("0.");
                                energy += " ";
                            }

                            isotopeInfos.Add(new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, energy));
                        }
                        IsotopeInfos = isotopeInfos;

                    }

                    if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                    {
                        //EnergySpectrum.Clear();
                        var espect = spectrum;

                        ObservableCollection<IsotopeInfo> isotopeInfos = new ObservableCollection<IsotopeInfo>();
                        List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(espect.FindPeaks(662, 50, 10, 2), 1, 662, 40, 10);
                        if (IsSpectrumAnalysisShow)
                        {
                            SnrSpetrum = new ObservableCollection<GraphData>(espect.SnrData);
                        }
                        else
                        {
                            SnrSpetrum = new ObservableCollection<GraphData>();
                        }
                        foreach (Isotope iso in DetectedIso)
                        {
                            string energy = "";
                            foreach (double e in iso.PeakEnergy)
                            {
                                energy += e.ToString("0.");
                                energy += " ";
                            }

                            isotopeInfos.Add(new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, energy));
                        }
                        IsotopeInfos = isotopeInfos;
                    }
                }

            }

            StatusUpdateMutex.ReleaseMutex();
        }
        private ObservableCollection<HistoEnergy> _energySpetrum = new ObservableCollection<HistoEnergy>();
        
        public ObservableCollection<HistoEnergy> EnergySpectrum
        {
            get { return _energySpetrum; }
            set
            {
                _energySpetrum = value;
                OnPropertyChanged(nameof(EnergySpectrum));
            }
        }
        public ObservableCollection<IsotopeInfo> _isotopeInfos = new ObservableCollection<IsotopeInfo>();
        public ObservableCollection<IsotopeInfo> IsotopeInfos
        {
            get
            {
                return _isotopeInfos;
            }
            set
            {
                _isotopeInfos = value;
                OnPropertyChanged(nameof(IsotopeInfos));
            }
        }
        private ObservableCollection<GraphData> _snrSpectrum = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> SnrSpetrum
        {
            get { return _snrSpectrum; }
            set
            {
                _snrSpectrum = value;
                OnPropertyChanged(nameof(SnrSpetrum));
            }
        }
        private eSpectrumCases _spectrumCases = eSpectrumCases.Scatter;
        public eSpectrumCases SpectrumCases
        {
            get { return _spectrumCases; }
            set
            {
                _spectrumCases = value;
                OnPropertyChanged(nameof(SpectrumCases));
            }
        }
        private int fpgaChannelNumber = 0;
        public int FpgaChannelNumber
        {
            get { return fpgaChannelNumber; }
            set
            {
                if (fpgaChannelNumber >= 0 && fpgaChannelNumber <= 15)
                {
                    fpgaChannelNumber = value;
                }
                OnPropertyChanged(nameof(FpgaChannelNumber));
            }
        }

        private bool isSpectrumAnalysisShow = false;
        public bool IsSpectrumAnalysisShow
        {
            get { return isSpectrumAnalysisShow; }
            set { 
                isSpectrumAnalysisShow = value; OnPropertyChanged(nameof(IsSpectrumAnalysisShow)); }
        }

        public override void Unhandle()
        {   
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
