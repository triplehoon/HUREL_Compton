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


        FPGAConrolPannelViewModel pannelViewModel = new FPGAConrolPannelViewModel();
        public MainViewModel()
        {
            FPGAControl = new CRUXELLLACC();
            FPGAVariable = FPGAControl.Variables;
            IsEnableOpenFPGAWindow = true;
            IsSessionAvailable = true;

            FPGAControl.USBChangeHandler += UpdateDeviceList;
            FPGAControl.USBChange();
            Messenger.Default.Register<WindowStateMessage>(this,
                (action) => ReceiveIsEnableOpenFPGAWindow(action)
                );
        }

        private RelayCommand mianWindowCloseCommand;
        public ICommand MianWindowCloseCommand
        {
            get { return (this.mianWindowCloseCommand) ?? (this.mianWindowCloseCommand = new RelayCommand(CloseMainWindow)); }
        }
        private void CloseMainWindow()
        {
            if(WService.Window != null)
            {
                if(WService.Window.IsLoaded)
                    WService.Window.Close();
            }            
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
        private RelayCommand openFPGAWindowCommand;
        public ICommand OpenFPGAWindowCommand
        {
            get { return (this.openFPGAWindowCommand) ?? (this.openFPGAWindowCommand = new RelayCommand(this.OpenFPGAWindow, this.CanOpenFPGAWindow)); }
        }

        private void OpenFPGAWindow()
        {

            WService.ShowFPGAWindow(pannelViewModel);
            //IsEnableOpenFPGAWindow = false;
        }
        private WindowService WService = new WindowService();
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
            set { isSessionStart = value; OnPropertyChanged(nameof(IsSessionStart)); }
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
            
            timer.Interval = TimeSpan.FromSeconds(1);
            timer.Tick += new EventHandler(TimerTick);
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

    class WindowService
    {
        public FPGAControlWindow Window;
        public void ShowFPGAWindow(object viewModel)
        {
            Window = new FPGAControlWindow();
            Window.DataContext = viewModel;
            Window.Show();
        }
    }

    public class TimespaneToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var timeSpan =(TimeSpan)value;
            return timeSpan.ToString();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    public class BoolToStringConverter : BoolToValueConverter<String> { }
    public class BoolToVisibilityConverter : BoolToValueConverter<Visibility> { }
    public class BoolToValueConverter<T> : IValueConverter
    {
        public T FalseValue { get; set; }
        public T TrueValue { get; set; }

        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value == null)
                return FalseValue;
            else
                return (bool)value ? TrueValue : FalseValue;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return value != null ? value.Equals(TrueValue) : false;
        }
    }
}
