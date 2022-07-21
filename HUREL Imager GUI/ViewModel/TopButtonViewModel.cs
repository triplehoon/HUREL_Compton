using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using log4net;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;

namespace HUREL_Imager_GUI.ViewModel
{
    public class TopButtonViewModel: ViewModelBase
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
        }
        private void updateSatus(object? obj, EventArgs args)
        {
            StartButtonEnabled = (LahgiApi.IsInitiate && LahgiApi.IsFpgaAvailable) && !LahgiApi.IsSessionStarting;
            IsRunning = LahgiApi.IsSessionStart;
            FileName = "";
            IsSaveBinary = LahgiApi.IsSavingBinary;
        }        

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= updateSatus;
            logger.Info("Unhandle StatusUpdate");
        }

        private string _fileName = "";
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
            await LahgiApi.StartSessionAsync("", _sessionCancle);
        }
        
        private AsyncCommand? _stopSessionCommand = null;
        public ICommand StopSessionCommand
        {
            get { return _stopSessionCommand ?? (_stopSessionCommand = new AsyncCommand(StopSession)); }
        }
        private async Task StopSession()
        {
            await Task.Run(() => { _sessionCancle?.Cancel(); });
        }
    
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

        }

    
    }
}
