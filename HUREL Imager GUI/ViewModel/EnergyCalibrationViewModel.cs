using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;

namespace HUREL_Imager_GUI.ViewModel
{
    class EnergyCalibrationViewModel : ViewModelBase
    {
        private SpectrumViewModel _spectrumViewModel;
        public SpectrumViewModel SpectrumViewModel 
        {
            get
            {
                return _spectrumViewModel;
            }
            set
            {
                _spectrumViewModel = value;
                OnPropertyChanged(nameof(SpectrumViewModel));
            }
        }

        public EnergyCalibrationViewModel()
        {
            _spectrumViewModel = new SpectrumViewModel();
            LahgiApi.StatusUpdate += StatusUpdate; UpdateStates();


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

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading || lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                {
                    UpdateStates();
                }
            }
            StatusUpdateMutex.ReleaseMutex();
        }
        private void UpdateStates()
        {
            if (LahgiApi.eEcalStates.Count() != 8)
            {
                return;
            }
            IsScatter0EcalOk = LahgiApi.eEcalStates[0] == eEcalState.Success;
            IsScatter0EcalUnknown = LahgiApi.eEcalStates[0] == eEcalState.Unknown;
            IsScatter0EcalChecking = LahgiApi.eEcalStates[0] != eEcalState.Success && !IsCalibrationReady;
            IsScatter1EcalOk = LahgiApi.eEcalStates[1] == eEcalState.Success;
            IsScatter1EcalUnknown = LahgiApi.eEcalStates[1] == eEcalState.Unknown;
            IsScatter1EcalChecking = LahgiApi.eEcalStates[1] != eEcalState.Success && !IsCalibrationReady;
            IsScatter2EcalOk = LahgiApi.eEcalStates[2] == eEcalState.Success;
            IsScatter2EcalUnknown = LahgiApi.eEcalStates[2] == eEcalState.Unknown;
            IsScatter2EcalChecking = LahgiApi.eEcalStates[2] != eEcalState.Success && !IsCalibrationReady;
            IsScatter3EcalOk = LahgiApi.eEcalStates[3] == eEcalState.Success;
            IsScatter3EcalUnknown = LahgiApi.eEcalStates[3] == eEcalState.Unknown;
            IsScatter3EcalChecking = LahgiApi.eEcalStates[3] != eEcalState.Success && !IsCalibrationReady;

            IsAbsorber0EcalOk = LahgiApi.eEcalStates[4] == eEcalState.Success;
            IsAbsorber0EcalUnknown = LahgiApi.eEcalStates[4] == eEcalState.Unknown;
            IsAbsorber0EcalChecking = LahgiApi.eEcalStates[4] != eEcalState.Success && !IsCalibrationReady;
            IsAbsorber1EcalOk = LahgiApi.eEcalStates[5] == eEcalState.Success;
            IsAbsorber1EcalUnknown = LahgiApi.eEcalStates[5] == eEcalState.Unknown;
            IsAbsorber1EcalChecking = LahgiApi.eEcalStates[5] != eEcalState.Success && !IsCalibrationReady;
            IsAbsorber2EcalOk = LahgiApi.eEcalStates[6] == eEcalState.Success;
            IsAbsorber2EcalUnknown = LahgiApi.eEcalStates[6] == eEcalState.Unknown;
            IsAbsorber2EcalChecking = LahgiApi.eEcalStates[6] != eEcalState.Success && !IsCalibrationReady;         
            IsAbsorber3EcalOk = LahgiApi.eEcalStates[7] == eEcalState.Success;
            IsAbsorber3EcalUnknown = LahgiApi.eEcalStates[7] == eEcalState.Unknown;
            IsAbsorber3EcalChecking = LahgiApi.eEcalStates[7] != eEcalState.Success && !IsCalibrationReady;
        }

        private AsyncCommand? startCalibrationCommand = null;
        public ICommand StartCalibrationCommand
        {
            get { return startCalibrationCommand ?? (startCalibrationCommand = new AsyncCommand(StartCalibration)); }
        }
        private async Task StartCalibration()
        {
            await Task.Run(() =>
            {
                IsCalibrationReady = false;
                UpdateStates();
                LahgiApi.CheckEcalState(MinRange, MaxRange);
                IsCalibrationReady = true;
                UpdateStates();
            });
        }

        private AsyncCommand? resetCalibrationCommand = null;
        public ICommand ResetCalibrationCommand
        {
            get { return resetCalibrationCommand ?? (resetCalibrationCommand = new AsyncCommand(ResetCalibration)); }
        }
        private async Task ResetCalibration()
        {
            await Task.Run(() =>
            {
                IsCalibrationReady = false;
                for (int i =0; i < 8; ++i)
                {
                    LahgiApi.eEcalStates[i] = eEcalState.Unknown;
                }
                
                IsCalibrationReady = true;
                UpdateStates();
            });
        }


        private bool isCalibrationReady = true;
        public bool IsCalibrationReady
        {
            get { return isCalibrationReady; }
            set { isCalibrationReady = value; OnPropertyChanged(nameof(IsCalibrationReady)); }
        }

        private double minRange = 1461 - 60;
        public double MinRange
        {
            get { return minRange; }
            set
            {
                minRange = value;
                OnPropertyChanged(nameof(MinRange));
            }
        }

        private double maxRange = 1461 + 60;
        public double MaxRange
        {
            get { return maxRange; }
            set
            {
                maxRange = value;
                OnPropertyChanged(nameof(MaxRange));
            }
        }

        #region EcalStatusViews

        private bool isScatter0EcalOk = false;
        public bool IsScatter0EcalOk
        {
            get { return isScatter0EcalOk; }
            set { isScatter0EcalOk = value; OnPropertyChanged(nameof(IsScatter0EcalOk)); }
        }
        private bool isScatter0EcalChecking = false;
        public bool IsScatter0EcalChecking
        {
            get { return isScatter0EcalChecking; }
            set { isScatter0EcalChecking = value; OnPropertyChanged(nameof(IsScatter0EcalChecking)); }
        }
        private bool isScatter0EcalUnknown = false;
        public bool IsScatter0EcalUnknown
        {
            get { return isScatter0EcalUnknown; }
            set { isScatter0EcalUnknown = value; OnPropertyChanged(nameof(IsScatter0EcalUnknown)); }
        }


        private bool isScatter1EcalOk = false;
        public bool IsScatter1EcalOk
        {
            get { return isScatter1EcalOk; }
            set { isScatter1EcalOk = value; OnPropertyChanged(nameof(IsScatter1EcalOk)); }
        }
        private bool isScatter1EcalChecking = false;
        public bool IsScatter1EcalChecking
        {
            get { return isScatter1EcalChecking; }
            set { isScatter1EcalChecking = value; OnPropertyChanged(nameof(IsScatter1EcalChecking)); }
        }
        private bool isScatter1EcalUnknown = false;
        public bool IsScatter1EcalUnknown
        {
            get { return isScatter1EcalUnknown; }
            set { isScatter1EcalUnknown = value; OnPropertyChanged(nameof(IsScatter1EcalUnknown)); }
        }

        
        private bool isScatter2EcalOk = false;
        public bool IsScatter2EcalOk
        {
            get { return isScatter2EcalOk; }
            set { isScatter2EcalOk = value; OnPropertyChanged(nameof(IsScatter2EcalOk)); }
        }
        private bool isScatter2EcalChecking = false;
        public bool IsScatter2EcalChecking
        {
            get { return isScatter2EcalChecking; }
            set { isScatter2EcalChecking = value; OnPropertyChanged(nameof(IsScatter2EcalChecking)); }
        }
        private bool isScatter2EcalUnknown = false;
        public bool IsScatter2EcalUnknown
        {
            get { return isScatter2EcalUnknown; }
            set { isScatter2EcalUnknown = value; OnPropertyChanged(nameof(IsScatter2EcalUnknown)); }
        }

        
        private bool isScatter3EcalOk = false;
        public bool IsScatter3EcalOk
        {
            get { return isScatter3EcalOk; }
            set { isScatter3EcalOk = value; OnPropertyChanged(nameof(IsScatter3EcalOk)); }
        }
        private bool isScatter3EcalChecking = false;
        public bool IsScatter3EcalChecking
        {
            get { return isScatter3EcalChecking; }
            set { isScatter3EcalChecking = value; OnPropertyChanged(nameof(IsScatter3EcalChecking)); }
        }
        private bool isScatter3EcalUnknown = false;
        public bool IsScatter3EcalUnknown
        {
            get { return isScatter3EcalUnknown; }
            set { isScatter3EcalUnknown = value; OnPropertyChanged(nameof(IsScatter3EcalUnknown)); }
        }
        
        private bool isAbsorber0EcalOk = false;
        public bool IsAbsorber0EcalOk
        {
            get { return isAbsorber0EcalOk; }
            set { isAbsorber0EcalOk = value; OnPropertyChanged(nameof(IsAbsorber0EcalOk)); }
        }
        private bool isAbsorber0EcalChecking = false;
        public bool IsAbsorber0EcalChecking
        {
            get { return isAbsorber0EcalChecking; }
            set { isAbsorber0EcalChecking = value; OnPropertyChanged(nameof(IsAbsorber0EcalChecking)); }
        }
        private bool isAbsorber0EcalUnknown = false;
        
        public bool IsAbsorber0EcalUnknown
        {
            get { return isAbsorber0EcalUnknown; }
            set { isAbsorber0EcalUnknown = value; OnPropertyChanged(nameof(IsAbsorber0EcalUnknown)); }
        }


        private bool isAbsorber1EcalOk = false;
        public bool IsAbsorber1EcalOk
        {
            get { return isAbsorber1EcalOk; }
            set { isAbsorber1EcalOk = value; OnPropertyChanged(nameof(IsAbsorber1EcalOk)); }
        }
        private bool isAbsorber1EcalChecking = false;
        public bool IsAbsorber1EcalChecking
        {
            get { return isAbsorber1EcalChecking; }
            set { isAbsorber1EcalChecking = value; OnPropertyChanged(nameof(IsAbsorber1EcalChecking)); }
        }
        private bool isAbsorber1EcalUnknown = false;
        public bool IsAbsorber1EcalUnknown
        {
            get { return isAbsorber1EcalUnknown; }
            set { isAbsorber1EcalUnknown = value; OnPropertyChanged(nameof(IsAbsorber1EcalUnknown)); }
        }

        private bool isAbsorber2EcalOk = false;
        public bool IsAbsorber2EcalOk
        {
            get { return isAbsorber2EcalOk; }
            set { isAbsorber2EcalOk = value; OnPropertyChanged(nameof(IsAbsorber2EcalOk)); }
        }
        private bool isAbsorber2EcalChecking = false;
        public bool IsAbsorber2EcalChecking
        {
            get { return isAbsorber2EcalChecking; }
            set { isAbsorber2EcalChecking = value; OnPropertyChanged(nameof(IsAbsorber2EcalChecking)); }
        }
        private bool isAbsorber2EcalUnknown = false;
        public bool IsAbsorber2EcalUnknown
        {
            get { return isAbsorber2EcalUnknown; }
            set { isAbsorber2EcalUnknown = value; OnPropertyChanged(nameof(IsAbsorber2EcalUnknown)); }
        }

        private bool isAbsorber3EcalOk = false;
        public bool IsAbsorber3EcalOk
        {
            get { return isAbsorber3EcalOk; }
            set { isAbsorber3EcalOk = value; OnPropertyChanged(nameof(IsAbsorber3EcalOk)); }
        }
        private bool isAbsorber3EcalChecking = false;
        public bool IsAbsorber3EcalChecking
        {
            get { return isAbsorber3EcalChecking; }
            set { isAbsorber3EcalChecking = value; OnPropertyChanged(nameof(IsAbsorber3EcalChecking)); }
        }
        private bool isAbsorber3EcalUnknown = false;
        public bool IsAbsorber3EcalUnknown
        {
            get { return isAbsorber3EcalUnknown; }
            set { isAbsorber3EcalUnknown = value; OnPropertyChanged(nameof(IsAbsorber3EcalUnknown)); }
        }


        #endregion






        public override void Unhandle()
        {
            SpectrumViewModel.Unhandle();
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
