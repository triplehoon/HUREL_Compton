using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
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
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            
            var histogram = LahgiApi.GetSpectrumEnergy(0).HistoEnergies;
            EnergySpectrum = new ObservableCollection<HistoEnergy>(histogram);

            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                {
                    EnergySpectrum.Clear();
                    var espect = LahgiApi.GetScatterSumSpectrumByTime(0);
                    
                    foreach(var e in espect.HistoEnergies)
                    {
                        EnergySpectrum.Add(e);
                    }
                    ObservableCollection<IsotopeInfo> isotopeInfos = new ObservableCollection<IsotopeInfo>();
                    List<Isotope>  DetectedIso =  PeakSearching.GetIsotopesFromPeaks(espect.FindPeaks(662, 40, 10, 10), 1, 662, 40, 10);
                    
                    foreach(Isotope iso in DetectedIso)
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

        public override void Unhandle()
        {   
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
