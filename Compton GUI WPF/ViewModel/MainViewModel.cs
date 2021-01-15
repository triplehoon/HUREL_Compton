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
    public partial class MainViewModel : INotifyPropertyChanged
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
            BPPointCloud = new HelixToolkit.Wpf.SharpDX.PointGeometry3D();

            RTPointCloudTask =Task.Run(() => GetRealTimePointCloud());

            //Messenger.Default.Register<WindowStateMessage>(this,
            //    (action) => ReceiveIsEnableOpenFPGAWindow(action)
            //    );


            ModuleInfoViewModels = new ObservableCollection<ModuleInfoViewModel>();

            for (int i = 0; i < 16; i++)
            {
                ModuleInfoViewModels.Add(new ModuleInfoViewModel());
            }

            InitiateLACC();
        }




        private RelayCommand mianWindowCloseCommand;
        public ICommand MianWindowCloseCommand
        {
            get { return (this.mianWindowCloseCommand) ?? (this.mianWindowCloseCommand = new RelayCommand(CloseMainWindow)); }
        }
        private void CloseMainWindow()
        {
            IsRealTimePointCloudOn = false;
            RTPointCloudTask.Wait();
            FPGAControl.SetVaribles(FPGAVariable);
            FPGAControl.Dispose();
           // rsControl.Dispose();
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
                Task taskDrawing = Task.Run(()=>DrawMLPEPositions());
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

        bool IsHistoGramTooSlow = false;
        public void DrawSpectrum()
        {
            if (IsHistoGramTooSlow)
            {
                return;
            }
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
            if (sw.ElapsedMilliseconds > 800)
                IsHistoGramTooSlow = true;
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
                    temp1.Add(new MlpePositionInfo(lmdatum.RelativeInteractionPoint3D.X - ModuleInfoViewModels[0].Offset.x, lmdatum.RelativeInteractionPoint3D.Y-ModuleInfoViewModels[0].Offset.y));
                }
            }
            AbsorberPositionData = temp1;
            //ScatterPositionData.Clear();
            var temp2 = new List<MlpePositionInfo>();
           
            foreach (var lmdata in scatterLMData)
            {
                foreach (var lmdatum in lmdata)
                {
                    temp2.Add(new MlpePositionInfo(lmdatum.RelativeInteractionPoint3D.X - ModuleInfoViewModels[8].Offset.x, lmdatum.RelativeInteractionPoint3D.Y - ModuleInfoViewModels[8].Offset.y));
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
                if(ModuleInfoViewModels[0].IsModuleSet && ModuleInfoViewModels[8].IsModuleSet)
                {

                    LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[8].Module);
                    IsLACCModuleInitiate = true;
                    initiating = false;
                    return;
                }

                var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = true, Order = new int[] { 0, 18, 1, 19, 2, 20, 11, 29, 10, 28, 9, 27, 3, 21, 4, 22, 5, 23, 14, 32, 13, 31, 12, 30, 6, 24, 7, 25, 8, 26, 17, 35, 16, 34, 15, 33 } };
                var scatterGain = new double[37]  {0.222287552011680,
                                                    0.208847009962622,
                                                    0.160835530297629,
                                                    0.350623925414967,
                                                    0.404254384165359,
                                                    0.173114587164014,
                                                    0.251973705604810,
                                                    0.287514444819041,
                                                    0.197784900587933,
                                                    0.113205828176507,
                                                    0.270750509096893,
                                                    0.324814715062910,
                                                    0.159132032020835,
                                                    0.179033913500545,
                                                    0.177842115156326,
                                                    0.108344401828327,
                                                    0.122989511991333,
                                                    0.333935336242705,
                                                    0.256168970319604,
                                                    0.115470302451087,
                                                    0.107632624571028,
                                                    0.343022471533058,
                                                    0.129540635721655,
                                                    0.184271389706723,
                                                    0.154867557833026,
                                                    0.183742374044755,
                                                    0.235758007303454,
                                                    0.183618330027555,
                                                    0.149858076110482,
                                                    0.404494624248347,
                                                    0.452139539299007,
                                                    0.304594382211978,
                                                    0.243182810827749,
                                                    0.343234564555548,
                                                    0.220940431055765,
                                                    0.370428100393800,
                                                    -19.3920305409253  };
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
                                                            new LACC_Module.ModuleOffset { x = 0, y = -0.22, z = 0 },
                                                            new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                            scatterGain,
                                                            pmtOrderInfo,
                                                            Path.Combine(LUTFolderDirectory, "MonoScatterLUT.csv"));

                Debug.WriteLine("Making Abosrober Module");
                TEST = "Making Absorber Module";
                ModuleInfoViewModels[8] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                            new LACC_Module.ModuleOffset { x = 0, y = -0.220, z = -0.250 },
                                                            new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                            absorberGain,
                                                            pmtOrderInfo,
                                                            Path.Combine(LUTFolderDirectory, "MonoAbsorberLUT.csv"));
                if (!ModuleInfoViewModels[0].IsModuleSet || !ModuleInfoViewModels[8].IsModuleSet)
                {

                    LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[8].Module);
                    IsLACCModuleInitiate = false;
                    initiating = false;
                    VMStatus = "Mono-Type Module Setting Failed";
                    return;
                }

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


            initiating = true;
            IsLACCModuleInitiate = false;
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
