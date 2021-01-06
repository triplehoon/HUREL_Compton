using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Data;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using Compton_GUI_WPF.View;
using GalaSoft.MvvmLight.Command;
using GalaSoft.MvvmLight.Messaging;
using HUREL.Compton;
using HUREL.Compton.LACC;
using MathNet.Numerics;
using MathNet.Numerics.Statistics;

namespace Compton_GUI_WPF.ViewModel
{
    public class MainViewModel : INotifyPropertyChanged
    {

        public static CRUXELLLACC FPGAControl;

        public static CRUXELLLACC.VariableInfo FPGAVariable;

        public static LACC_Control LACC_Control_Static;

        private ModuleInfo selectedModuleInfo = ModuleInfo.Mono;
        public ModuleInfo SelecteModuleInfo 
        { get { return selectedModuleInfo; } 
            set { 
                selectedModuleInfo = value; 
            } 
        }

        private string CurrentPath = Directory.GetCurrentDirectory();
        
        public MainViewModel()
        {
            FPGAControl = new CRUXELLLACC();
            FPGAVariable = FPGAControl.Variables;
            IsEnableOpenFPGAWindow = true;
            IsSessionAvailable = false;

            FPGAControl.USBChangeHandler += UpdateDeviceList;
            FPGAControl.USBChange();
            spectrumHistoModels = new ObservableCollection<List<SpectrumHisto.SpectrumHistoModel>>();
            for(int i = 0; i <16; i++)
            {
                spectrumHistoModels.Add(new List<SpectrumHisto.SpectrumHistoModel>());
            }
            //Messenger.Default.Register<WindowStateMessage>(this,
            //    (action) => ReceiveIsEnableOpenFPGAWindow(action)
            //    );
            
            InitiateLACC();
        }




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
                        IsSessionStart = true;
                        StartTimer();
                        IsAddingListModeData = true;
                        AddListModeDataTask = new Task(() => AddListModeData());
                        AddListModeDataTask.Start();
                        
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
                timer.Stop();
                RecordTimeSpan = TimeSpan.Zero;
            }



            IsSessionAvailable = true;
        }

        private Task AddListModeDataTask;
        private bool IsAddingListModeData;
        private void AddListModeData()
        {
            short[] check1;
            short[] check2 = new short[256];
            while (IsAddingListModeData)
            {
                short[] item;
                while(FPGAControl.ShortArrayQueue.TryTake(out item))
                {
                    check1 = item;
                    if(check1==check2)
                    {
                        Debug.WriteLine("CEHK");
                    }
                    check2 = item;
                    LACC_Control_Static.AddListModeData(item, Matrix3D.Identity);
                }
            }
        }

        public void DrawSpectrum()
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();
            for (int i = 0; i < 16; i++)
            {
                var SelectedESpect = (from selESpect in LACC_Control_Static.EnergySpect
                                      where selESpect.Channel == i
                                      select selESpect.Energy).ToList();
                SpectrumHisto histo = new SpectrumHisto(SelectedESpect, 405, 0, 2100);
                SpectrumHistoModels[i] = histo.SpectrumData;
            }
            sw.Stop();
            Debug.WriteLine("DrawSpectrums elapsed time is " + sw.ElapsedMilliseconds + " ms");
        }

        private ObservableCollection<List<SpectrumHisto.SpectrumHistoModel>> spectrumHistoModels;
        public ObservableCollection<List<SpectrumHisto.SpectrumHistoModel>> SpectrumHistoModels
        {
            get { return spectrumHistoModels; }
        }

        public class SpectrumHisto
        {
            public List<SpectrumHistoModel> SpectrumData = new List<SpectrumHistoModel>();
            public SpectrumHisto(IEnumerable<double> data, int nbuckets, double lower, double upper)
            {
                Histogram hist = new Histogram(data, nbuckets, lower, upper);
                for (int i = 0; i < hist.BucketCount; i++)
                {
                    SpectrumData.Add(new SpectrumHistoModel { LowerBound = hist[i].LowerBound, Count = hist[i].Count });
                }

            }

            public class SpectrumHistoModel
            {
                public double LowerBound { get; set; }
                public double Count { get; set; }
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
            Task.Run(()=>DataUpdate());
        }

        private void TimerTick(object sender, EventArgs e)
        {
            RecordTimeSpan = RecordTimeSpan.Add(TimeSpan.FromSeconds(1));
        }

        private void DataUpdate()
        {
            while (IsSessionStart)
            {
                DrawSpectrum();
                Thread.Sleep(1000);
            }
            Debug.WriteLine("DataUpdate End");
        }

        #endregion

        #region LACC Module Setting

        private bool isLACCModuleInitiate = false;
        public bool IsLACCModuleInitiate
        {
            get { return isLACCModuleInitiate; }
            set
            {
                isLACCModuleInitiate = value;                
                OnPropertyChanged(nameof(IsLACCModuleInitiate));
                OnPropertyChanged(nameof(IsSessionAvailable));
            }
        }

        private string LUTFolderDirectory = Path.Combine(Directory.GetCurrentDirectory(),"LUT Files");
        public void InitiateMonoType()
        {
            Task.Run(() =>
            {
                var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = true, Order = new int[] { 0, 18, 1, 19, 2, 20, 11, 29, 12, 28, 9, 27, 3, 21, 4, 22, 5, 23, 14, 32, 13, 31, 12, 30, 6, 24, 7, 25, 8, 26, 17, 35, 16, 34, 15, 33 } };

                var scatterGain = new double[37]  { 0.229400822535143,
                                                0.194663785680398,
                                                0.184236289538727,
                                                0.328845970032704,
                                                0.388535540257414,
                                                0.182997191802852,
                                                0.256568926962897,
                                                0.261898167575063,
                                                0.200476805251476,
                                                0.107149915166777,
                                                0.258711656044783,
                                                0.317571489375082,
                                                0.153968089385871,
                                                0.175575768904058,
                                                0.175069094879092,
                                                0.111338944874489,
                                                0.126300762020813,
                                                0.310296386792488,
                                                0.271149150631817,
                                                0.110569664442744,
                                                0.109709278149893,
                                                0.312856624047890,
                                                0.135521237098968,
                                                0.193823476297495,
                                                0.147240054519398,
                                                0.199701252506580,
                                                0.222085079797251,
                                                0.186253277487988,
                                                0.163246220676073,
                                                0.363372707108992,
                                                0.451220095983549,
                                                0.294538914503081,
                                                0.234470528667482,
                                                0.330946205527829,
                                                0.201129108512092,
                                                0.399876618388626,
                                                -19.1899788970696 };

                var absorberGain = new double[37] { 0.609106629714565,
                                                0.408225309758093,
                                                0.461847452592639,
                                                0.420864773543207,
                                                0.406298442910974,
                                                0.556871972880209,
                                                0.427062526383404,
                                                0.529611054266539,
                                                0.385468424382990,
                                                0.248421318082802,
                                                0.399864947053825,
                                                0.425536517980407,
                                                0.339859200857057,
                                                0.398740664113444,
                                                0.464483368090175,
                                                0.403390895135249,
                                                0.298422129818660,
                                                0.553180476402401,
                                                0.642667635434905,
                                                0.358890089937244,
                                                0.464030776465580,
                                                0.445993103539891,
                                                0.273774321638299,
                                                0.214176752360862,
                                                0.621807100373737,
                                                0.356965167293123,
                                                0.376619470434398,
                                                0.289744640131841,
                                                0.369076302531657,
                                                0.674687609116932,
                                                0.639591093149570,
                                                0.556966464257456,
                                                0.651793451901132,
                                                0.363504215341530,
                                                0.662096134248347,
                                                0.599963606291628,
                                                -20.6402542760799 };

                Debug.WriteLine("Making Scatter Module");
                TEST = "Making Scatter Module";
                ModuleInfoViewModels[0] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                            new LACC_Module.ModuleOffet { x = 0, y = 0, z = 0 },
                                                            new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                            scatterGain,
                                                            pmtOrderInfo,
                                                            Path.Combine(LUTFolderDirectory, "MonoScatterLUT.csv"));

                Debug.WriteLine("Making Abosrober Module");
                TEST = "Making Absorber Module";
                ModuleInfoViewModels[8] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                            new LACC_Module.ModuleOffet { x = 0, y = 0, z = -150 },
                                                            new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                            absorberGain,
                                                            pmtOrderInfo,
                                                            Path.Combine(LUTFolderDirectory, "MonoAbsorberLUT.csv"));

                LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[8].Module);
                IsLACCModuleInitiate = true;
                initiating = false;
            });
            
        }

        public void InitiateSingleHeadQuadType()
        {
            IsLACCModuleInitiate = true;
        }

        public void InitiateDualHeadQuadType()
        {
            IsLACCModuleInitiate = true;
        }

        public ObservableCollection<ModuleInfoViewModel> ModuleInfoViewModels { get; set; } //16 Channels


        private RelayCommand initiateLACCommand;
        public ICommand InitiateLACCommand
        {
            get { return (this.initiateLACCommand) ?? (this.initiateLACCommand = new RelayCommand(InitiateLACC)); }
        }
        private bool initiating;
        private void InitiateLACC()
        {
            if (initiating == true)
                return;

            ModuleInfoViewModels = new ObservableCollection<ModuleInfoViewModel>();

            for (int i = 0; i < 16; i++)
            {
                ModuleInfoViewModels.Add(new ModuleInfoViewModel());
            }

            initiating = true;
            IsLACCModuleInitiate = false;
            InitiateLACC();
            switch (this.selectedModuleInfo)
            {
                case ModuleInfo.Mono:
                    InitiateMonoType();
                    break;
                case ModuleInfo.QuadSingleHead:
                    InitiateSingleHeadQuadType();
                    break;
                case ModuleInfo.QuadDualHead:
                    InitiateDualHeadQuadType();
                    break;
            }
        }


        private RelayCommand<string> ecalInfoChangedCommand;
        public ICommand EcalInfoChangedCommand
        {
            get { return (this.ecalInfoChangedCommand) ?? (this.ecalInfoChangedCommand = new RelayCommand<string>(EcalInfoChanged)); }
        }
        private void EcalInfoChanged(string a)
        {
            Debug.WriteLine("Channel num is " + a);
            try
            {
                int channelNum=Convert.ToInt32(a);
                ModuleInfoViewModels[channelNum].Fitting();                
            }
            catch
            {
                Debug.WriteLine("EcalInfoChanged parameter is wrong");
            }

        }

        #endregion


        private RelayCommand<object> testCommand;
        public ICommand TestCommand
        {
            get { return (this.testCommand) ?? (this.testCommand = new RelayCommand<object>(TestFunction)); }
        }
        private void TestFunction(object obj)
        {
            var a = obj;
            Debug.WriteLine("TestFuction");
        }

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

    


    


}
