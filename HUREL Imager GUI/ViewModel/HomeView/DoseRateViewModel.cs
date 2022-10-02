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
    
    public class DoseRateViewModel : ViewModelBase
    {

        public DoseRateViewModel()
        {
            LahgiApi.StatusUpdate += StatusUpdate;
           
        }
        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {               
            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }
            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                {
                    var espect = LahgiApi.GetScatterSumSpectrumByTime(0);

                    (double dose, double std) = espect.GetAmbientDose(DoseCalcTime);
                    if (dose > 0.01)
                    {
                        DoseLogScale = Math.Log10(dose * 100);
                    }
                    else
                    {
                        DoseLogScale = 0;
                    }

                }
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                {
                    var espect =LahgiApi.GetScatterSumSpectrumByTime(doseCalcTime);
                    (double dose, double std) = espect.GetAmbientDose(DoseCalcTime);
                    if (dose > 0.01)
                    {
                        DoseLogScale = Math.Log10(dose * 100);
                    }
                    else
                    {
                        DoseLogScale = 0;
                    }

                }
            }
            StatusUpdateMutex.ReleaseMutex();
        }

        private double doseLogScale = Math.Log10(30 * 100);
        public double DoseLogScale
        {
            get { return doseLogScale; }
            set { 
                doseLogScale = value; 
                OnPropertyChanged(nameof(DoseLogScale)); 
            }
        }

        private uint doseCalcTime = 5;
        public uint DoseCalcTime
        {
            get 
            { 
                return doseCalcTime; 
            }
            set 
            { 
                doseCalcTime = value; 
                OnPropertyChanged(nameof(DoseCalcTime)); 
            }
        }



        public override void Unhandle()
        {   
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
