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


        private async Task InitiateCZTAsync()
        {
            if (!SRE3021API.IsTCPOpen || !SRE3021API.IsUDPOpen)
            {
                return;
            }

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
                Message = "Starting CZT";
                IsCZTRunning = true;
                SRE3021API.StartAcqusition();
                System.Windows.Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        StopCZTCommmand.RaiseCanExecuteChanged();
                        StartCZTCommmand.RaiseCanExecuteChanged();
                    }));



                Message = "CZT is Started";
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
        bool IsCZTRunning = false;
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
            Message = "CZT is stop";
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


        private string message;
        public string Message
        {
            get
            {
                return message;
            }
            set
            {
                message = value;
                OnPropertyChanged(nameof(Message));
            }
        }

        static void ProcessImgData(SRE3021ImageData imgData)
        {
            int interactionX = -1;
            int interactionY = -1;
            int interactionPotins = 0;
            int backgroundNoise = 0;
            for (int X = 0; X < 11; ++X)
            {
                for (int Y = 0; Y < 11; ++Y)
                {
                    if (imgData.AnodeTiming[X, Y] > 50)
                    {
                        ++interactionPotins;
                        if (interactionPotins == 2)
                        {
                            return;
                        }
                        interactionX = X;
                        interactionY = Y;
                    }
                    else
                    {
                        backgroundNoise += imgData.AnodeValue[X, Y];
                    }

                }
            }
            if (interactionX == -1)
            {
                return;
            }
            backgroundNoise = backgroundNoise / 118;

            CZTSpectrumEnergy.AddEnergy(imgData.AnodeValue[interactionX, interactionY] * p1 + p2);

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

        static private SpectrumEnergy CZTSpectrumEnergy = new SpectrumEnergy(2, 1500);

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

