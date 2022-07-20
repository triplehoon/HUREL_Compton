using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using System;
using System;
using System;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    public class SpectrumViewModel : ViewModelBase
    {

        public SpectrumViewModel()
        {
            LahgiApi.StatusUpdate += StatusUpdate;
            EnergySpectrum = new ObservableCollection<HistoEnergy>();
            Random random = new Random();
            for (int i = 0; i < 3000; i++)
            {
                EnergySpectrum.Add(new HistoEnergy(i, random.Next(5000)));
            }
        }
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        { 

        }

        private ObservableCollection<HistoEnergy> _energySpectrum = new ObservableCollection<HistoEnergy>();
        public ObservableCollection<HistoEnergy> EnergySpectrum { get; set; }
        public override void Unhandle()
        {   
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
