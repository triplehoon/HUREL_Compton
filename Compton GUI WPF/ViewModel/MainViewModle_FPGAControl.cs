using AsyncAwaitBestPractices;
using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton.LACC;
using HUREL.Compton;
using MathNet.Numerics.Statistics;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using System.IO;

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel
    {



        public static CRUXELLLACC FPGAControl;
       
        public static CRUXELLLACC.VariableInfo FPGAVariable;


        #region FPGA Setting

        private CRUXELLLACC.DeviceInfo selectDevice;
        public CRUXELLLACC.DeviceInfo SelectDevice
        {
            get
            {
                return selectDevice;
            }
            set
            {
                FPGAControl.SelectedDevice = value;
                selectDevice = value;
                OnPropertyChanged(nameof(SelectDevice));
            }
        }

        private ObservableCollection<CRUXELLLACC.DeviceInfo> deviceInfos = new ObservableCollection<CRUXELLLACC.DeviceInfo>();
        public ObservableCollection<CRUXELLLACC.DeviceInfo> DeviceInfos
        {
            get
            {
                return deviceInfos;
            }
            set
            {
                deviceInfos = value;
                OnPropertyChanged(nameof(DeviceInfos));
            }
        }

        private void UpdateDeviceList(object sender, EventArgs e)
        {
            DeviceInfos = new ObservableCollection<CRUXELLLACC.DeviceInfo>(FPGAControl.DeviceList);
            SelectDevice = FPGAControl.SelectedDevice;
            if (DeviceInfos.Count > 0)
            {
                IsSessionAvailable = true;
            }
            else
            {
                IsSessionAvailable = false;
            }
        }



        private string fileName;
        public string FileName
        {
            get
            {
                fileName = MainViewModel.FPGAVariable.FileName;
                return fileName;
            }
            set
            {
                fileName = value;
                MainViewModel.FPGAVariable.FileName = value;
                OnPropertyChanged(nameof(FileName));
            }
        }


        #endregion


        #region Start or Stop USB

        private AsyncCommand startorStopSessionCommand;
        public ICommand StartorStopSessionCommand
        {
            get { return (this.startorStopSessionCommand) ?? (this.startorStopSessionCommand = new AsyncCommand(StartorStopSessionAsync, CanExecuteStartorStopSession)); }
        }
        private bool CanExecuteStartorStopSession(object arg)
        {
            bool allSessionAvailable = IsLACCModuleInitiate && IsSessionAvailable;
            return allSessionAvailable;
        }
        private async Task StartorStopSessionAsync()
        {                        
            if (MeasurementTime == "")
                return;
            IsSessionAvailable = false;

            if (fileName != "")
            {
                IsSavingBinaryFile = true;
            }
            else
            {
                IsSavingBinaryFile = false;
            }

            if (!IsSessionStart)
            {
                if (!FPGAControl.SetVaribles(FPGAVariable))
                {
                    VMStatus = "Please configure FPGA.";
                    IsSessionAvailable = true;
                    return;
                }
                else
                {
                    VMStatus = "FPGA setting Start";

                    string status = "";

                    bool isFPGAStart = await Task.Run(() => FPGAControl.Start_usb(out status)).ConfigureAwait(false);
                    if (isFPGAStart) 
                    {
                        IsSessionStart = true;
                        IsRealTimeImageReconOn = true;
                        await Task.Run(()=>StartTimer());
                        IsAddingListModeData = true;
                        AddListModeDataTaskAsync = Task.Run(() => AddListModeData());
                        RealTimeImageReconTaskAsync = Task.Run(() => RealTimeImageRecon());
                    }
                    VMStatus = status;
                }
            }
            else // Stoppping Usb
            {
                VMStatus = await FPGAControl.Stop_usb();
                IsSessionStart = false;
                IsAddingListModeData = false;
                await AddListModeDataTaskAsync.ConfigureAwait(false);
                FPGADispatchTimer.Stop();
                IsRealTimeImageReconOn = false;
                await RealTimeImageReconTaskAsync;
                await Task.Run(() => DrawMLPEPositions()).ConfigureAwait(false);
                IsRealTimeImageReconOn = true;
                if (IsSavingBinaryFile)
                {
                    VMStatus = "Saving CSV file";
                    LACC_Control_Static.SaveListmodeData(Path.GetDirectoryName(FPGAControl.FileMainPath), fileName + "T265Offset_" + T265ToLACCOffset.X + "_" +T265ToLACCOffset.Y +"_" + T265ToLACCOffset.Z + "_");
                    SaveCurrentPointCloud(Path.GetDirectoryName(FPGAControl.FileMainPath), fileName);
                    VMStatus = "Done saving CSV file";
                }                
            }
            IsSessionAvailable = true;
        }

        private bool isMLPEOn = false;
        public bool IsMLPEOn
        {
            get { return isMLPEOn; }
            set { isMLPEOn = value; OnPropertyChanged(nameof(IsMLPEOn)); }
        }

        private List<AddListModeDataEchk> Echks = new List<AddListModeDataEchk>();

        private int minMLPE_Energy = 0;
        public int MinMLPE_Energy
        {
            get {
                Echks = new List<AddListModeDataEchk> { new AddListModeDataEchk(minMLPE_Energy, maxMLPE_Energy) };

                return minMLPE_Energy; 
            }
            set 
            {
                Echks = new List<AddListModeDataEchk> { new AddListModeDataEchk(value, maxMLPE_Energy) };
                minMLPE_Energy = value; 
                OnPropertyChanged(nameof(MinMLPE_Energy));
            }
        }
        private int maxMLPE_Energy = 1500;
        public int MaxMLPE_Energy
        {
            get {
                Echks = new List<AddListModeDataEchk> { new AddListModeDataEchk(minMLPE_Energy, maxMLPE_Energy) };
                return maxMLPE_Energy; }
            set
            {
                Echks = new List<AddListModeDataEchk> { new AddListModeDataEchk(minMLPE_Energy, value) };
                maxMLPE_Energy = value;
                OnPropertyChanged(nameof(MaxMLPE_Energy));
            }
        }


        private Task AddListModeDataTaskAsync;
        private bool IsAddingListModeData;
        private void AddListModeData()
        {           

            LACC_Control_Static.ResetLMData();
            while (IsAddingListModeData)
            {
                ushort[] item;
                while (FPGAControl.ShortArrayQueue.TryTake(out item))
                {
                    
                    LACC_Control_Static.AddListModeData(item, CurrentSystemTranformation, Echks, IsMLPEOn);
                    //Thread.Sleep(0);
                    
                }
            }
        }



        private bool isSavingBinaryFile = false;
        public bool IsSavingBinaryFile
        {
            get
            {                
                return isSavingBinaryFile;
            }
            set
            {
                isSavingBinaryFile = value;
                FPGAControl.IsSavingBinaryData = value;
                OnPropertyChanged(nameof(IsSavingBinaryFile));
            }
        }

        private bool isSessionAvailable;
        public bool IsSessionAvailable
        {
            get 
            { 
                return isSessionAvailable;
            }
            set
            {
                isSessionAvailable = value;
                OnPropertyChanged(nameof(IsSessionAvailable));
                Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        ((AsyncCommand)StartorStopSessionCommand).RaiseCanExecuteChanged();
                    }));
            }
        }
        

        private bool isSessionStart = false;
        public bool IsSessionStart
        {
            get { return isSessionStart; }
            set { isSessionStart = value; IsSesstionStop = !value; OnPropertyChanged(nameof(IsSessionStart)); }
        }
       
        private bool isSessionStop = true;
        public bool IsSesstionStop
        {
            get { return isSessionStop; }
            set { isSessionStop = value; OnPropertyChanged(nameof(IsSesstionStop)); }
        }
         
        private string measurementTime;
        public string MeasurementTime
        {
            get
            {
                return measurementTime;
            }
            set
            {
                try
                {
                    FPGAVariable.RecordTime0x0a = Convert.ToInt32(value);
                    MeasurementTimeSpan = TimeSpan.FromSeconds(FPGAVariable.RecordTime0x0a);
                }
                catch(FormatException e) 
                {
                    Debug.WriteLine(e);
                    MeasurementTimeSpan = TimeSpan.Zero; 
                }
                measurementTime = value;
                OnPropertyChanged(nameof(MeasurementTime));
            }
        }
       
        private TimeSpan measurementTimeSpan = TimeSpan.Zero;
        public TimeSpan MeasurementTimeSpan
        {
            get { return measurementTimeSpan; }
            set
            {
                measurementTimeSpan = value;
                OnPropertyChanged(nameof(MeasurementTimeSpan));
            }
        }

        private TimeSpan recordTimeSpan = TimeSpan.Zero;
        public TimeSpan RecordTimeSpan
        {
            get { return recordTimeSpan; }
            set
            {
                recordTimeSpan = value;                
                OnPropertyChanged(nameof(RecordTimeSpan));
            }
        }

        private System.Timers.Timer FPGADispatchTimer = new System.Timers.Timer();
        private void StartTimer()
        {
            RecordTimeSpan = TimeSpan.Zero;
            FPGADispatchTimer = new System.Timers.Timer();
            FPGADispatchTimer.Interval = 1000;
            FPGADispatchTimer.Elapsed += OnTimerTick;
            FPGADispatchTimer.Start();
            Task.Run(() => DataUpdate());
        }

        private void OnTimerTick(object sender, EventArgs e)
        {
            RecordTimeSpan = RecordTimeSpan.Add(TimeSpan.FromSeconds(1));
            
            if (IsSessionStart && MeasurementTimeSpan == RecordTimeSpan)
            {
                FPGADispatchTimer.Stop();                
                StartorStopSessionAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
                recordTimeSpan = TimeSpan.Zero;
                FPGADispatchTimer.Dispose();
                VMStatus = "Done!";
            }
        }

        

        #endregion

    }
}
