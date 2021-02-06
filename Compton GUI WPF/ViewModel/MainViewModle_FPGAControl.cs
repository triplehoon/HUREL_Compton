using GalaSoft.MvvmLight.Command;
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
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

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

        private RelayCommand startorStopSessionCommand;
        public ICommand StartorStopSessionCommand
        {
            get { return (this.startorStopSessionCommand) ?? (this.startorStopSessionCommand = new RelayCommand(StartorStopSession, IsSessionAvailable)); }
        }
        private void StartorStopSession()
        {
            IsHistoGramTooSlow = false;
            if (MeasurementTime == "")
                return;
            IsSessionAvailable = false;
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

                    string status;
                    if (FPGAControl.Start_usb(out status))
                    {
                        IsSessionStart = true;
                        StartTimer();
                        IsAddingListModeData = true;
                        AddListModeDataTask = new Task(() => AddListModeData());
                        RealTimeImageReconTask = new Task(() => RealTimeImageRecon());
                        AddListModeDataTask.Start();
                        RealTimeImageReconTask.Start();

                    }
                    VMStatus = status;
                }
            }
            else
            {
                VMStatus = FPGAControl.Stop_usb();
                IsSessionStart = false;
                IsAddingListModeData = false;
                AddListModeDataTask.Wait();
                FPGADispatchTimer.Stop();
                RecordTimeSpan = TimeSpan.Zero;
                Task taskDrawing = Task.Run(() => DrawMLPEPositions());
            }



            IsSessionAvailable = true;
        }

        private bool isMLPEOn = false;
        public bool IsMLPEOn
        {
            get { return isMLPEOn; }
            set { isMLPEOn = value; OnPropertyChanged(nameof(IsMLPEOn)); }
        }

        private int minMLPE_Energy = 0;
        public int MinMLPE_Energy
        {
            get { return minMLPE_Energy; }
            set { minMLPE_Energy = value; OnPropertyChanged(nameof(MinMLPE_Energy)); }
        }
        private int maxMLPE_Energy = 100;
        public int MaxMLPE_Energy
        {
            get { return maxMLPE_Energy; }
            set
            {
                maxMLPE_Energy = value;
                OnPropertyChanged(nameof(MaxMLPE_Energy));
            }
        }


        private Task AddListModeDataTask;
        private bool IsAddingListModeData;
        private void AddListModeData()
        {
            short[] check1;
            short[] check2 = new short[256];
            LACC_Control_Static.ResetLMData();
            var Identity = Matrix3D.Identity;
            while (IsAddingListModeData)
            {
                short[] item;
                while (FPGAControl.ShortArrayQueue.TryTake(out item))
                {
                    check1 = item;
                    if (check1 == check2)
                    {
                        Debug.WriteLine("CEHK");
                    }

                    if (!IsSLAMOn)
                    {
                        LACC_Control_Static.AddListModeData(item, Identity, isMLPEOn, minMLPE_Energy, maxMLPE_Energy);
                    }  
                    else
                    {
                        LACC_Control_Static.AddListModeData(item, CurrentSystemTranformation, isMLPEOn, minMLPE_Energy, maxMLPE_Energy);
                    }
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
                bool allSessionAvailable = IsLACCModuleInitiate && isSessionAvailable;
                return allSessionAvailable;
            }
            set
            {
                isSessionAvailable = value;
                OnPropertyChanged(nameof(IsSessionAvailable));
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
                catch { MeasurementTimeSpan = TimeSpan.Zero; }
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
                if (IsSessionStart && MeasurementTimeSpan == value)
                {
                    FPGADispatchTimer.Stop();
                    Task stopSession = new Task(StartorStopSession);
                    stopSession.Start();
                    VMStatus = "Wait For Stop";
                    stopSession.Wait();
                    recordTimeSpan = TimeSpan.Zero;
                    VMStatus = "Done!";
                }
                OnPropertyChanged(nameof(RecordTimeSpan));
            }
        }

        private DispatcherTimer FPGADispatchTimer = new DispatcherTimer();
        private void StartTimer()
        {
            FPGADispatchTimer = new DispatcherTimer();
            FPGADispatchTimer.Interval = TimeSpan.FromSeconds(1);
            FPGADispatchTimer.Tick += new EventHandler(TimerTick);
            FPGADispatchTimer.Start();
            Task.Run(() => DataUpdate());
        }

        private void TimerTick(object sender, EventArgs e)
        {
            RecordTimeSpan = RecordTimeSpan.Add(TimeSpan.FromSeconds(1));
        }

        

        #endregion

    }
}
