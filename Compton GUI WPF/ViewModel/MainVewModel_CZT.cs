using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL.Compton.CZT;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel : INotifyPropertyChanged
    {

        private AsyncCommand startOrStopCZTCommand;
        public IAsyncCommand StartOrStopCZTCommand
        {
            get { return startOrStopCZTCommand ?? (startOrStopCZTCommand = new AsyncCommand(StartOrStopCZT, CanExecuteStartorStopCZT)); }
        }

        private bool CanExecuteStartorStopCZT(object arg)
        {
            if (!SRE3021API.IsAPIAvailable)
            {
                return false;
            }
            if (isStartingCZT)
            {
                return false;
            }
            return true;

        }
        private async Task StartOrStopCZT()
        {
            if (!SRE3021API.IsAPIAvailable)
            {
                return;
            }
            if (!IsCZTRunning)
            {
                await StartCZT();
            }
            else
            {
                await StopCZT();
            }
        }


        private TimeSpan cztRunTime = new TimeSpan();

        private AsyncCommand startCZTCommmand;
        public IAsyncCommand StartCZTCommmand
        {
            get { return startCZTCommmand ?? (startCZTCommmand = new AsyncCommand(StartCZT, CanExecuteStartCZT)); }
        }
        private bool CanExecuteStartCZT(object arg)
        {
            return !IsCZTRunning;
        }

        bool isStartingCZT = false;

        private async Task StartCZT()
        {
            await Task.Run(() =>
            {
                SRE3021API.ResetSpectrumEnergy();
                VMStatus = "Starting CZT";
                isStartingCZT = true;             
                System.Windows.Application.Current.Dispatcher.Invoke(DispatcherPriority.ApplicationIdle,
                new Action(() =>
                {
                    StopCZTCommmand.RaiseCanExecuteChanged();
                    StartCZTCommmand.RaiseCanExecuteChanged();
                    StartOrStopCZTCommand.RaiseCanExecuteChanged();
                }));

                SRE3021API.StartAcqusition();

                VMStatus = "CZT is Started";
                isStartingCZT = false;
                IsCZTRunning = true;
                System.Windows.Application.Current.Dispatcher.Invoke(DispatcherPriority.ApplicationIdle,
                new Action(() =>
                {
                    StopCZTCommmand.RaiseCanExecuteChanged();
                    StartCZTCommmand.RaiseCanExecuteChanged();
                    StartOrStopCZTCommand.RaiseCanExecuteChanged();
                }));

                DateTime startTime = DateTime.Now;
                while (IsCZTRunning)
                {
                    List<HistoEnergy> cztESpectrum = SRE3021API.GetSpectrumEnergy.HistoEnergies;
                    SpectrumHistoCZT = new ObservableCollection<HistoEnergy>(cztESpectrum);
                    List<double> peaks = SRE3021API.GetSpectrumEnergyIsoFind.FindPeaks(662, 16.5f, 0.1f, 3.0f);
                    List<Isotope> Isotopes = SpectrumEnergyNasa.GetIsotopesFromPeaks(peaks, 16.5f, 0.1f, 3.0f);
                    List<HistoEnergy> peakArea = new List<HistoEnergy>();
                    foreach(Isotope iso in Isotopes)
                    {
                        foreach (HistoEnergy e in cztESpectrum)
                        {
                            if (e.Energy > iso.PeakEnergy - 50 && e.Energy < iso.PeakEnergy + 50)
                            {
                                peakArea.Add(e);
                            }
                        }
                    }
                    CZTFindIsotopes = new ObservableCollection<Isotope>(Isotopes);
                    SpectrumCZTPeak = new ObservableCollection<HistoEnergy>(peakArea);
                    Thread.Sleep(100);
                }
                cztRunTime = DateTime.Now - startTime;
            });
        }



        private ObservableCollection<Isotope> cztFindIsotopes = new ObservableCollection<Isotope>();
        public ObservableCollection<Isotope> CZTFindIsotopes
        {
            get
            {
                return cztFindIsotopes;
            }
            set
            {
                cztFindIsotopes = value;
                OnPropertyChanged(nameof(CZTFindIsotopes));
            }
        }


        public bool isCZTRunning = false;
        public bool IsCZTRunning
        {
            get
            {
                return isCZTRunning;
            }
            set
            {
                isCZTRunning = value;
                OnPropertyChanged(nameof(IsCZTRunning));
            }
        }
        private AsyncCommand stopCZTCommmand;
        public IAsyncCommand StopCZTCommmand
        {
            get { return stopCZTCommmand ?? (stopCZTCommmand = new AsyncCommand(StopCZT, CanExecuteStopCZT)); }
        }

        private bool CanExecuteStopCZT(object arg)
        {
            return IsCZTRunning;
        }

        private async Task StopCZT()
        {
            await Task.Run(() =>
            {
                IsCZTRunning = false;
                isStartingCZT = true;
                System.Windows.Application.Current.Dispatcher.Invoke(DispatcherPriority.ApplicationIdle, new Action(() =>
                {
                    StopCZTCommmand.RaiseCanExecuteChanged();
                    StartCZTCommmand.RaiseCanExecuteChanged();
                    StartOrStopCZTCommand.RaiseCanExecuteChanged();
                }));
                SRE3021API.StopAcqusition();
            });
            VMStatus = "CZT is stop";
            isStartingCZT = false;
            System.Windows.Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        StartCZTCommmand.RaiseCanExecuteChanged();
                        StopCZTCommmand.RaiseCanExecuteChanged();
                        StartOrStopCZTCommand.RaiseCanExecuteChanged();
                    }));

        }

        private AsyncCommand resetCZTSpectrumCommand;
        public IAsyncCommand ResetCZTSpectrumCommand
        {
            get { return resetCZTSpectrumCommand ?? (resetCZTSpectrumCommand = new AsyncCommand(ResetCZTSpectrum)); }
        }



        private async Task ResetCZTSpectrum()
        {
            await Task.Run(() =>
            {
                SRE3021API.ResetSpectrumEnergy();
                SpectrumHistoCZT = new ObservableCollection<HistoEnergy>();
                SpectrumCZTPeak = new ObservableCollection<HistoEnergy>();
                CZTFindIsotopes = new ObservableCollection<Isotope>();
            });
        }

        private AsyncCommand setHVCommand;
        public IAsyncCommand SetHVCommand
        {
            get { return setHVCommand ?? (setHVCommand = new AsyncCommand(SetHV)); }
        }
        private async Task SetHV()
        {
            await Task.Run(() =>
            {
                SRE3021API.SetHighVoltage(hvValue, 10, 100);
            });
        }
        private int hvValue;
        public int HVValue
        {
            get
            {
                return hvValue;
            }
            set
            {
                hvValue = value;
                OnPropertyChanged(nameof(HVValue));
            }
        }


        private ObservableCollection<HistoEnergy> spectrumCZTPeak = new ObservableCollection<HistoEnergy>();
        public ObservableCollection<HistoEnergy> SpectrumCZTPeak
        {
            get
            {
                return spectrumCZTPeak;
            }
            set
            {
                spectrumCZTPeak = value;
                OnPropertyChanged(nameof(SpectrumCZTPeak));
            }
        }


        private ObservableCollection<HistoEnergy> spectrumHistoCZT = new ObservableCollection<HistoEnergy>();
        public ObservableCollection<HistoEnergy> SpectrumHistoCZT
        {
            get
            {
                return spectrumHistoCZT;
            }
            set
            {
                spectrumHistoCZT = value;
                OnPropertyChanged(nameof(SpectrumHistoCZT));
            }
        }


    }
}
