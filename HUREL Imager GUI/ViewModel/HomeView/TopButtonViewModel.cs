using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using log4net;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using System.Windows.Input;

namespace HUREL_Imager_GUI.ViewModel
{
    public class TopButtonViewModel : ViewModelBase
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(TopButtonViewModel));

        private bool _startButtonEnabled = true;
        public bool StartButtonEnabled
        {
            get { return _startButtonEnabled; }
            set { _startButtonEnabled = value; OnPropertyChanged(nameof(StartButtonEnabled)); }
        }

        private bool _isRunning = false;
        public bool IsRunning
        {
            get { return _isRunning; }
            set { _isRunning = value; OnPropertyChanged(nameof(IsRunning)); }
        }

        public TopButtonViewModel()
        {
            LahgiApi.StatusUpdate += updateSatus;
            updateSatus(null, EventArgs.Empty);
            logger.Info("TopButtonViewModel Loaded");

            System.Timers.Timer timer = new System.Timers.Timer();
            timer.Interval = 500;
            timer.Elapsed += UpdateTimerInvoker;
            timer.Start();
        }

        private void UpdateTimerInvoker(object? sender, ElapsedEventArgs e)
        {
            //231017 sbkwon : 경과 시간 측정
            if (IsRunning is true)
            {
                ElapsedTime = (uint)(LahgiApi.SessionStopwatch.ElapsedMilliseconds * 0.001);
            }
        }

        private void updateSatus(object? obj, EventArgs args)
        {
            StartButtonEnabled = (LahgiApi.IsInitiate && LahgiApi.IsFpgaAvailable) && !LahgiApi.IsSessionStarting;
            IsRunning = LahgiApi.IsSessionStart;
            IsSaveBinary = LahgiApi.IsSavingBinary;
        }

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= updateSatus;
            StopSession();
            logger.Info("Unhandle StatusUpdate");
        }

        private string _fileName = "Default";
        public string FileName
        {
            get { return _fileName; }
            set
            {
                _fileName = value;
                OnPropertyChanged(nameof(FileName));
            }
        }

        private CancellationTokenSource? _sessionCancle;
        private AsyncCommand? startSessionCommand = null;
        public ICommand StartSessionCommand
        {
            get { return startSessionCommand ?? (startSessionCommand = new AsyncCommand(StartSession)); }
        }
        private async Task StartSession()
        {
            _sessionCancle = new CancellationTokenSource();

            //231017 sbkwon : 측정 시간 자동 종료 사용
            if (IsInfinite is false)
                _sessionCancle.CancelAfter(MeasurementTime * 1000);

            await LahgiApi.StartSessionAsync(FileName, _sessionCancle);
        }

        private AsyncCommand? _stopSessionCommand = null;
        public ICommand StopSessionCommand
        {
            get { return _stopSessionCommand ?? (_stopSessionCommand = new AsyncCommand(StopSession)); }

        }
        private Task StopSession() => Task.Run(() =>
        {
            LahgiApi.IsSessionStarting = false; _sessionCancle?.Cancel();

            //231017 sbkwon : spectrum capture
            //ImgCapture imgCapture = new ImgCapture(980, 110, 630, 580);
            //imgCapture.SetPath(saveFileName + "_Spectrum.png");
            //imgCapture.DoCaptureImage();
        });

        //test code
        //public Stopwatch SessionStopwatch = new Stopwatch();

        //public async Task StartSessionAsync(string fileName, CancellationTokenSource tokenSource)
        //{
        //    await Task.Run(() => { AddListModeData(tokenSource); });
        //}

        //public void AddListModeData(CancellationTokenSource tokenSource)
        //{
        //    Trace.WriteLine("Thread Start");
        //    SessionStopwatch.Restart();
        //    while (true) 
        //    {
        //        if (tokenSource.IsCancellationRequested)
        //            break;

        //        Thread.Sleep(500);

        //        Trace.WriteLine($"Out Thread {ElapsedTime} ");
        //    }

        //    SessionStopwatch.Stop();
        //    IsRunning = false;
        //    Trace.WriteLine("Thread End");
        //}
        //test code

        private bool _isSaveBinary;
        public bool IsSaveBinary
        {
            get { return _isSaveBinary; }
            set
            {
                _isSaveBinary = value;
                LahgiApi.IsSavingBinary = value;
                OnPropertyChanged(nameof(IsSaveBinary));
            }
        }

        private AsyncCommand? _loadDataCommand;
        public ICommand LoadDataCommand
        {
            get { return _loadDataCommand ?? (_loadDataCommand = new AsyncCommand(LoadData)); }
        }
        private async Task LoadData()
        {
            OpenFileDialog dlg = new OpenFileDialog();
            dlg.Filter = "ply files(*.ply)| *.ply";
            dlg.Multiselect = false;
            dlg.Title = "Select pointcloud file";

            if (dlg.ShowDialog() == true)
            {
                await Task.Run(() => LahgiApi.LoadPlyFile(dlg.FileName));
            }
            else
            {
                return;
            }
            LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);

            dlg = new OpenFileDialog();
            dlg.Filter = "csv files(*.csv)| *.csv";
            dlg.Multiselect = false;
            dlg.Title = "Select list mode data file";

            if (dlg.ShowDialog() == true)
            {
                await Task.Run(() => LahgiApi.LoadListModeData(dlg.FileName));
            }
            else
            {
                return;
            }


            LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);

        }

        private AsyncCommand? _testFuctionCommand;
        public ICommand TestFunctionCommand
        {
            get { return _testFuctionCommand ?? (_testFuctionCommand = new AsyncCommand(TestFunction)); }
        }
        private async Task TestFunction()
        {
            TestFunctionCommand.CanExecute(false);

            await Task.Run(() => {
                LahgiApi.TestAddingListModeData(1000_000);


            }
            );

            TestFunctionCommand.CanExecute(true);

        }

        //231017 sbkwon : 측정 시간 자동 종료 사용 여부
        private bool _isInfinite = true;
        public bool IsInfinite
        {
            get
            {
                return _isInfinite;
            }
            set
            {
                _isInfinite = value;
                OnPropertyChanged(nameof(IsInfinite));
            }
        }

        //231016 sbkwon : 측정 시간
        private int _measurementTime = 0;
        public int MeasurementTime
        {
            get { return _measurementTime; }
            set
            {
                _measurementTime = value;

                OnPropertyChanged(nameof(MeasurementTime));
            }
        }

        //231016 sbkwon : 경과 시간
        private uint _elapsedTime = 0;
        public uint ElapsedTime
        {
            get { return _elapsedTime; }
            set
            {
                _elapsedTime = value;
                LahgiApi.ElapsedTime = value;

                OnPropertyChanged(nameof(ElapsedTime));
            }
        }
    }
}
