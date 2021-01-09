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
                DrawMLPEPositions();
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
            set { 
                maxMLPE_Energy = value; 
                OnPropertyChanged(nameof(MaxMLPE_Energy)); }
        }


        private Task AddListModeDataTask;
        private bool IsAddingListModeData;
        private void AddListModeData()
        {
            short[] check1;
            short[] check2 = new short[256];
            LACC_Control_Static.ResetLMData();
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
                    LACC_Control_Static.AddListModeData(item, Matrix3D.Identity,isMLPEOn,minMLPE_Energy,maxMLPE_Energy);
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

        public void DrawMLPEPositions()
        {
            var absorberLMData = (from LMData in LACC_Control_Static.ListedLMData
                                  select LMData.AbsorberLMDataInfos).ToList();
            var scatterLMData = (from LMData in LACC_Control_Static.ListedLMData
                                  select LMData.ScatterLMDataInfos).ToList();

            Stopwatch sw = new Stopwatch();
            sw.Start();
            //AbsorberPositionData.Clear();
            var temp1 = new List<MlpePositionInfo>();
            foreach (var lmdata in absorberLMData)
            {
                foreach (var lmdatum in lmdata)
                {
                    temp1.Add(new MlpePositionInfo(lmdatum.TransformedInteractionPoint3D.X, lmdatum.TransformedInteractionPoint3D.Y));
                }
            }
            AbsorberPositionData = temp1;
            //ScatterPositionData.Clear();
            var temp2 = new List<MlpePositionInfo>();
           
            foreach (var lmdata in scatterLMData)
            {
                foreach (var lmdatum in lmdata)
                {
                    temp2.Add(new MlpePositionInfo(lmdatum.TransformedInteractionPoint3D.X, lmdatum.TransformedInteractionPoint3D.Y));
                }
            }
            ScatterPositionData = temp2;
            sw.Stop();
            Debug.WriteLine("DrawSpectrums elapsed time is " + sw.ElapsedMilliseconds + " ms");




        }
        public record MlpePositionInfo(double X, double Y);

        private List<MlpePositionInfo> absorberPositionData = new List<MlpePositionInfo>();
        public List<MlpePositionInfo> AbsorberPositionData
        {
            get { return absorberPositionData; }
            set { absorberPositionData = value; OnPropertyChanged(nameof(AbsorberPositionData)); }
        }

        private List<MlpePositionInfo> scatterPositionData = new List<MlpePositionInfo>();
        public List<MlpePositionInfo> ScatterPositionData 
        { get { return scatterPositionData; } 
           set { scatterPositionData = value; OnPropertyChanged(nameof(ScatterPositionData)); }
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

                var scatterGain = new double[37]  {0.207583486310340,
0.197378617654247 ,
0.134143497193450 ,
0.350795612936240 ,
0.365945323071866 ,
0.169767500915951 ,
0.250029018792162 ,
0.257436772257243 ,
0.180262246045204 ,
0.122534704286570 ,
0.243337498004493 ,
0.315226084314086 ,
0.141924535715014 ,
0.173801413648601 ,
0.156002799209616 ,
0.107450094661035 ,
0.114231575985511 ,
0.304549612177718 ,
0.266789891182305 ,
0.100228059111417 ,
0.104187271753870 ,
0.332072906087907 ,
0.125365155392261 ,
0.157917759109811 ,
0.154776351938155 ,
0.173901827136405 ,
0.225933092606352 ,
0.160176422879037 ,
0.146448386501010 ,
0.358212260296200 ,
0.480430155445668 ,
0.264496676194317 ,
0.227795361479290 ,
0.339992966549734 ,
0.189348098309348 ,
0.321090890371102 ,
1.82850526010004  };
                var absorberGain = new double[37] { 0.547118426,
                                                    0.423998687,
                                                    0.426206901,
                                                    0.408303161,
                                                    0.410912616,
                                                    0.557610406,
                                                    0.444274915,
                                                    0.513597437,
                                                    0.371204235,
                                                    0.279542663,
                                                    0.355811448,
                                                    0.405420482,
                                                    0.346096898,
                                                    0.358584417,
                                                    0.439085018,
                                                    0.381797553,
                                                    0.324406816,
                                                    0.486858039,
                                                    0.604246889,
                                                    0.357470801,
                                                    0.473510762,
                                                    0.437615232,
                                                    0.289740831,
                                                    0.200805523,
                                                    0.57044647 ,
                                                    0.383966989,
                                                    0.322863322,
                                                    0.3249445  ,
                                                    0.329417575,
                                                    0.743689349,
                                                    0.485597352,
                                                    0.733393991,
                                                    0.453444903,
                                                    0.433348959,
                                                    0.754890154,
                                                    0.5538563  ,
                                                    -0.430917509
                                                    };

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
