using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Threading;
using Compton_GUI_WPF.View;
using GalaSoft.MvvmLight.Command;
using GalaSoft.MvvmLight.Messaging;
using HUREL.Compton;


namespace Compton_GUI_WPF.ViewModel
{
    public class MainViewModel : INotifyPropertyChanged
    {

        public static CRUXELLLACC FPGAControl;

        public static CRUXELLLACC.VariableInfo FPGAVariable;

       
        public MainViewModel()
        {
            FPGAControl = new CRUXELLLACC();
            FPGAVariable = FPGAControl.Variables;
            IsEnableOpenFPGAWindow = true;
            IsSessionAvailable = true;

            FPGAControl.USBChangeHandler += UpdateDeviceList;
            FPGAControl.USBChange();
            //Messenger.Default.Register<WindowStateMessage>(this,
            //    (action) => ReceiveIsEnableOpenFPGAWindow(action)
            //    );

            ecalInfos = new ObservableCollection<EcalInfo>();
            this.GenerateEcalInfos();
        }

        #region Test DataGrid

        private ObservableCollection<EcalInfo> ecalInfos;
        public ObservableCollection<EcalInfo> EcalInfos
        {
            get { return ecalInfos; }
            set 
            { 
                ecalInfos = value; 
                OnPropertyChanged(nameof(EcalInfo)); 
            }
        }


        private void GenerateEcalInfos()
        {
            EcalInfos.Add(new EcalInfo(662, 0, "Cs-137"));
            EcalInfos.Add(new EcalInfo(60, 0, "Am-241"));
            EcalInfos.Add(new EcalInfo(511, 0, "Na-22"));
            EcalInfos.Add(new EcalInfo(1275, 0, "Na-22"));
            EcalInfos.Add(new EcalInfo(1173, 0, "Co-60"));
            EcalInfos.Add(new EcalInfo(1332, 0, "Co-60"));
        }

        #endregion


        private RelayCommand mianWindowCloseCommand;
        public ICommand MianWindowCloseCommand
        {
            get { return (this.mianWindowCloseCommand) ?? (this.mianWindowCloseCommand = new RelayCommand(CloseMainWindow)); }
        }
        private void CloseMainWindow()
        {
   
            FPGAControl.SetVaribles(FPGAVariable);
            FPGAControl.Dispose();
        }

        private string test;
        public string TEST
        {
            get { return test; }
            set { test = value; OnPropertyChanged(nameof(TEST)); }
        }



        #region FPGA Setting Window
        private object ReceiveIsEnableOpenFPGAWindow(WindowStateMessage action)
        {
            IsEnableOpenFPGAWindow = !action.state;
            return null;
        }
        private bool isEnableOpenFPGAWindow;
        public bool IsEnableOpenFPGAWindow
        {
            get { return isEnableOpenFPGAWindow; }
            set { isEnableOpenFPGAWindow = value; OnPropertyChanged(nameof(IsEnableOpenFPGAWindow)); }
        }
 
        public bool CanOpenFPGAWindow()
        {
            return IsEnableOpenFPGAWindow;
        }
        #endregion

        #region Setting

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
                        StartTimer();
                        IsSessionStart = true;
                    }
                    VMStatus = status;
                }
            }
            else
            {
                
                VMStatus = FPGAControl.Stop_usb();
                IsSessionStart = false;
                timer.Stop();
                RecordTimeSpan = TimeSpan.Zero;
            }



            IsSessionAvailable = true;
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
                    timer.Stop();
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

        private DispatcherTimer timer = new DispatcherTimer();
        private void StartTimer()
        {
            timer = new DispatcherTimer();
            timer.Interval = TimeSpan.FromSeconds(1);
            timer.Tick += new EventHandler(TimerTick);
            timer.Start();
        }

        private void TimerTick (object sender, EventArgs e)
        {
            RecordTimeSpan = RecordTimeSpan.Add(TimeSpan.FromSeconds(1));
        }


        #endregion

        private string vmStatus;
        public string VMStatus 
        { 
            get { return vmStatus; }
            set { vmStatus = value; OnPropertyChanged(nameof(VMStatus)); }
        }



        private RelayCommand<TextCompositionEventArgs> tbPrivewTextInputOnlyNumericCommand;
        public ICommand TBPrivewTextInputOnlyNumericCommand
        {
            get
            {
                return this.tbPrivewTextInputOnlyNumericCommand ??
                    (this.tbPrivewTextInputOnlyNumericCommand = new RelayCommand<TextCompositionEventArgs>(TBPrivewTextInputOnlyNumeric));
            }
        }
        public void TBPrivewTextInputOnlyNumeric(TextCompositionEventArgs e)
        {
            Regex regex = new Regex("[^0-9]+");
            e.Handled = regex.IsMatch(e.Text);
            Debug.WriteLine(e.Text);
        }




        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }


    }

    public class EcalInfo
    {

        double trueEnergy;
        double mesuredEnergy;
        string sourceName;



        public double TrueEnergy
        {
            get { return trueEnergy; }
            set
            {
                trueEnergy = value;

            }
        }


        public double MeasuredEnergy
        {
            get { return mesuredEnergy; }
            set
            {
                mesuredEnergy = value;

            }
        }

        public string SourceName
        {
            get { return sourceName; }
            set { sourceName = value; }
        }




        public EcalInfo(double trueEnergy, double measuredEnergy = 0, string sourceName = "Unkown")
        {
            this.TrueEnergy = trueEnergy;
            this.MeasuredEnergy = measuredEnergy;
            this.SourceName = sourceName;
        }

    }


}
