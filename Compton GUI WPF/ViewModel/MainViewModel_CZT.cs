using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Threading;
using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL.Compton.CZT;

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel : INotifyPropertyChanged
    {
        private bool isCZTInitiate = false;
        public bool IsCZTInitiate
        {
            get
            {
                return isCZTInitiate;
            }
            set
            {
                isCZTInitiate = value;
                OnPropertyChanged(nameof(IsCZTInitiate));
            }
        }

        private AsyncCommand initiateCZTCommand;
        public IAsyncCommand InitiateCZTCommand
        {
            get { return initiateCZTCommand ?? (initiateCZTCommand = new AsyncCommand(InitiateCZTAsync)); }
        }
        private async Task InitiateCZTAsync()
        {
           try
            {
                SRE3021API.OpenUDPPort();
                SRE3021API.OpenTCPPort();
                SRE3021API.CheckAPI();
            }
            catch
            {
                return;
            }
            if (IsCZTInitiate == true)
            {
                return;
            }
            IsCZTInitiate = true;
            SRE3021API.IMGDataEventRecieved += ProcessImgData;

            await Task.Run(() =>
            {
                SRE3021API.CheckBaseline();
                for (int i = 0; i < 11; ++i)
                {
                    for (int j = 0; j < 11; ++j)
                    {
                        Console.WriteLine($"X: {i}, Y: {j}, Baseline {SRE3021API.AnodeValueBaseline[i, j]} // TimingBaseline{SRE3021API.AnodeTimingBaseline[i, j]}");

                    }
                }
            });
        }


        private AsyncCommand startCZTCommmand;
        public IAsyncCommand StartCZTCommmand
        {
            get { return startCZTCommmand ?? (startCZTCommmand = new AsyncCommand(StartCZT, CanExecuteStartCZT)); }
        }
        private bool CanExecuteStartCZT(object arg)
        {
            return !IsCZTRunning;
        }

        private async Task StartCZT()
        {
            await Task.Run(() =>
            {
                CZTMessage = "Starting CZT";
                IsCZTRunning = true;
                SRE3021API.StartAcqusition();
                System.Windows.Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        StopCZTCommmand.RaiseCanExecuteChanged();
                        StartCZTCommmand.RaiseCanExecuteChanged();
                    }));

                CZTSpectrumEnergy = new SpectrumEnergy(5, 1500);

                CZTMessage = "CZT is Started";
                while (IsCZTRunning)
                {
                    for (int i = 0; i < 1000; ++i)
                    {
                        Thread.Sleep(1);
                        DataCount = DataCountStatic;
                    }
                    CZTSpectrum = new ObservableCollection<HistoEnergy>(CZTSpectrumEnergy.HistoEnergies);
                }
            });

        }
        private bool isCZTRunning = false;
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
                SRE3021API.StopAcqusition();
            });
            CZTMessage = "CZT is stop";
            System.Windows.Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        StartCZTCommmand.RaiseCanExecuteChanged();
                        StopCZTCommmand.RaiseCanExecuteChanged();
                    }));

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


        private string cztmessage;
        public string CZTMessage
        {
            get
            {
                return cztmessage;
            }
            set
            {
                cztmessage = value;
                OnPropertyChanged(nameof(CZTMessage));
            }
        }
        
        static void ProcessImgData(SRE3021ImageData imgData)
        {
            List<int> interactionX = new List<int>();
            List<int> interactionY = new List<int>();
            int interactionPotins = 0;
            int backgroundNoise = 0;
            for (int X = 0; X < 11; ++X)
            {
                for (int Y = 0; Y < 11; ++Y)
                {
                    if (imgData.AnodeTiming[X, Y] > 50)
                    {
                        ++interactionPotins;

                        interactionX.Add(X);
                        interactionY.Add(Y);
                    }
                    else
                    {
                        backgroundNoise += imgData.AnodeValue[X, Y];
                    }

                }
            }
            if (interactionX.Count == 0)
            {
                return;
            }
            backgroundNoise = backgroundNoise / 118;

            if (interactionPotins == 1)
            {
                CZTSpectrumEnergy.AddEnergy(imgData.AnodeValue[interactionX[0], interactionY[0]] * p1 + p2);
            }
            
            //if (interactionPotins == 2)
            //{
            //    CZTSpectrumEnergy.AddEnergy(imgData.AnodeValue[interactionX[0], interactionY[0]] * p1 + p2 + imgData.AnodeValue[interactionX[1], interactionY[1]] * p1 + p2);
            //}

            DataCountStatic++;
        }

        static int DataCountStatic = 0;

        const double p1 = 0.2987;
        const double p2 = 49.72;

        private int dataCount = 0;
        public int DataCount
        {
            get { return dataCount; }
            set
            {
                dataCount = value;
                OnPropertyChanged(nameof(DataCount));
            }
        }

        static private SpectrumEnergy CZTSpectrumEnergy = new SpectrumEnergy(5, 1500);

        private ObservableCollection<HistoEnergy> cztSpectrum = new ObservableCollection<HistoEnergy>();
        public ObservableCollection<HistoEnergy> CZTSpectrum
        {
            get
            {
                return cztSpectrum;
            }
            set
            {
                cztSpectrum = value;
                OnPropertyChanged(nameof(CZTSpectrum));
            }
        }


    }


}

