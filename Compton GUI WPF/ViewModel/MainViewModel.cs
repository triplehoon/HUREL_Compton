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
using AsyncAwaitBestPractices;
using AsyncAwaitBestPractices.MVVM;
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

        public static LACC_Control LACC_Control_Static;


        /// <summary>
        /// Contructor
        /// </summary>
        public MainViewModel()
        {
            FPGAControl = new CRUXELLLACC();
            FPGAVariable = FPGAControl.Variables;

            IsSessionAvailable = false;

            FPGAControl.USBChangeHandler += UpdateDeviceList;
            FPGAControl.USBChange();            
           

            //RTPointCloudTask =Task.Run(() => GetRealTimePointCloud());

            ModuleInfoViewModels = new ObservableCollection<ModuleInfoViewModel>();
            ModuleEnergySpectrums = new List<ObservableCollection<HistoEnergy>>();

            for (int i = 0; i < 16; i++)
            {
                ModuleEnergySpectrums.Add(new ObservableCollection<HistoEnergy>());
                ModuleInfoViewModels.Add(new ModuleInfoViewModel());
            }

            
            InitiateRealsenseAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            InitiateLACCAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));          
            TestFunction("").SafeFireAndForget(onException: ex => Debug.WriteLine(ex));

        }





        private ModuleInfo selectedModuleInfo = ModuleInfo.Mono;
        public ModuleInfo SelecteModuleInfo
        {
            get { return selectedModuleInfo; }
            set
            {
                selectedModuleInfo = value;
            }
        }


        private readonly string CurrentPath = Directory.GetCurrentDirectory();
        
        private AsyncCommand mianWindowCloseCommand;
        public ICommand MianWindowCloseCommand
        {
            get { return (this.mianWindowCloseCommand) ?? (this.mianWindowCloseCommand = new AsyncCommand(CloseMainWindow)); }
        }
        private async Task CloseMainWindow()
        {
            await StopRealsensePipeline().ConfigureAwait(false);
            await StopSLAM().ConfigureAwait(false);
            FPGAControl.SetVaribles(FPGAVariable);
            await FPGAControl.Dispose().ConfigureAwait(false);
            // rsControl.Dispose();
        }

        #region Draw Graph

        private void DataUpdate()
        {
            LACC_Control.debugCountAbsorber = new int[4] { 0, 0, 0, 0 };
            LACC_Control.debugCountScatter = new int[4] { 0, 0, 0, 0 };
            while (IsSessionStart)
            {
                DrawSpectrum();
                if (RecordTimeSpan.TotalSeconds > 10 && IsMLPEOn)
                {
                    //ResetSpectrumCommand.Execute(null);
                }                
                Thread.Sleep(1000);
            }
            Debug.WriteLine("DataUpdate End");

            Debug.WriteLine($"||| {LACC_Control.debugCountScatter[0]} ||| \n" +
                $"||| {LACC_Control.debugCountScatter[1]} ||| \n" +
                $"||| {LACC_Control.debugCountScatter[2]} ||| \n" +
                $"||| {LACC_Control.debugCountScatter[3]} ||| \n" +
                $"||| {LACC_Control.debugCountAbsorber[0]} ||| \n" +
                $"||| {LACC_Control.debugCountAbsorber[1]} ||| \n" +
                $"||| {LACC_Control.debugCountAbsorber[2]} ||| \n" +
                $"||| {LACC_Control.debugCountAbsorber[3]} ||| \n" +            
                $"");

        }
        public void DrawMLPEPositions()
        {
            if(IsMLPEOn == false)
            {
                return;
            }
            var absorberLMData = (from LMData in LACC_Control_Static.ListedLMData
                                  select LMData.AbsorberLMDataInfo).ToList();
            var scatterLMData = (from LMData in LACC_Control_Static.ListedLMData
                                 select LMData.ScatterLMDataInfo).ToList();

            Stopwatch sw = new Stopwatch();
            sw.Start();
            //AbsorberPositionData.Clear();
            var temp1 = new List<MlpePositionInfo>();
            foreach (var lmdata in absorberLMData)
            {
               if (lmdata == null)
               {
                   continue;
               }
               temp1.Add(new MlpePositionInfo(lmdata.RelativeInteractionPoint3D.X - T265ToLACCOffset.X, lmdata.RelativeInteractionPoint3D.Y - T265ToLACCOffset.Y));
               if (sw.ElapsedMilliseconds > 5000)
                {
                    break;
                }
            }
            AbsorberPositionData = temp1;
            //ScatterPositionData.Clear();
            var temp2 = new List<MlpePositionInfo>();

            foreach (var lmdata in scatterLMData)
            {
                if (lmdata == null)
                {
                    continue;
                }
                temp2.Add(new MlpePositionInfo(lmdata.RelativeInteractionPoint3D.X - T265ToLACCOffset.X, lmdata.RelativeInteractionPoint3D.Y - T265ToLACCOffset.Y));      
                
            }
            ScatterPositionData = temp2;
            sw.Stop();
           /// Debug.WriteLine("DrawSpectrums elapsed time is " + sw.ElapsedMilliseconds + " ms");
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
        {
            get { return scatterPositionData; }
            set { scatterPositionData = value; OnPropertyChanged(nameof(ScatterPositionData)); }
        }

        private AsyncCommand resetSepctrumCommand;
        public ICommand ResetSpectrumCommand
        {
            get { return (this.resetSepctrumCommand) ?? (this.resetSepctrumCommand = new AsyncCommand(ResetSpectrum)); }
        }
        private async Task ResetSpectrum()
        {
            await Task.Run(()=>LACC_Control_Static.ResetModuleEnergy());
        }

        public List<ObservableCollection<HistoEnergy>> ModuleEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> TestModuleEnergySpectrums { get; set; } 
        public void DrawSpectrum()
        {

            Stopwatch sw = new Stopwatch();
            sw.Start();
            int i = 0;
            if (IsLACCModuleInitiate == false)
            {
                return;
            }
            foreach (var data in LACC_Control_Static.LACC_Scatter_Modules)
            {
                ModuleEnergySpectrums[i] = new ObservableCollection<HistoEnergy>(data.SpectrumEnergy.HistoEnergies);
                OnPropertyChanged($"ModuleEnergySpectrums[{i}]");
                ++i;
            }
            i = 8;
            foreach (var data in LACC_Control_Static.LACC_Absorber_Modules)
            {
                ModuleEnergySpectrums[i] = new ObservableCollection<HistoEnergy>(data.SpectrumEnergy.HistoEnergies);
                OnPropertyChanged($"ModuleEnergySpectrums[{i}]");
                ++i;
            }
            OnPropertyChanged(nameof(ModuleEnergySpectrums));
            sw.Stop();
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
                Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        ((AsyncCommand)StartorStopSessionCommand).RaiseCanExecuteChanged();
                    }));
            }
        }

        private string LUTFolderDirectory = Path.Combine(Directory.GetCurrentDirectory(), "LUT Files");
        public void InitiateMonoType()
        {
            if (ModuleInfoViewModels[0].IsModuleSet && ModuleInfoViewModels[8].IsModuleSet)
            {

                LACC_Control_Static = new LACC_Control(ModuleInfoViewModels[0].Module, ModuleInfoViewModels[8].Module);
                IsLACCModuleInitiate = true;
                initiating = false;
                return;
            }

            var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = true, Order = new int[] { 0, 18, 1, 19, 2, 20, 11, 29, 10, 28, 9, 27, 3, 21, 4, 22, 5, 23, 14, 32, 13, 31, 12, 30, 6, 24, 7, 25, 8, 26, 17, 35, 16, 34, 15, 33 } };
            var scatterGain = new double[37]  { 0.200202731289930,
                0.190847222851550,   
                0.147996767565226,   
                0.359272699508623,   
                0.380243854240363,   
                0.169438763693375,   
                0.260476989727359,   
                0.274391453156910,   
                0.189471051291874,   
                0.114906931283501,   
                0.267956229918823,  
                0.322394274283856,   
                0.141667889901735,   
                0.168952781466941,   
                0.165559902613287,   
                0.111282387790912,   
                0.117841380598527,   
                0.310289390015741,   
                0.263141175554965,   
                0.105508769408180,   
                0.106677383006991,   
                0.336681789556159,   
                0.144091312300634,   
                0.173594270231844,   
                0.146426135203099,   
                0.174465416271953,   
                0.225146576166695,   
                0.175160859021928,   
                0.150709221155378,   
                0.378957270256709,   
                0.436355561830887,  
                0.293429537791822,  
                0.233246114361345,   
                0.320804207839602,   
                0.201330179693305,   
                0.351916695505510,   
                -33.5949592925280};
            var absorberGain = new double[37] { 0.516987034703535,
                0.381808481930781,
                0.401856613052412,
                0.411022444545442,
                0.388933593107869,   
                0.504883190381881,   
                0.403453224083484,   
                0.465114575809726,   
                0.375488171302086,   
                0.259035801679925,   
                0.361252449617899,   
                0.360960472553622,   
                0.346661022233274,   
                0.342171956318829,   
                0.428219999513722,   
                0.356226263872230,   
                0.287960942572674,   
                0.466154485102795,   
                0.587678972804617,   
                0.352147996520178,   
                0.481069877539962,   
                0.390939193572481,   
                0.305342718045117,   
                0.188897008903444,   
                0.537049817397682,   
                0.336902729914176,   
                0.302555761053709,   
                0.319113891201546,   
                0.294327565173938,   
                0.658909281896846,   
                0.533791330564325,   
                0.663797060022686,   
                0.459134314936778,   
                0.390724235635081,   
                0.728624922786835,   
                0.504278527608377,   
                -32.3134123545437
            };

            Debug.WriteLine("Making Scatter Module");
            VMStatus = "Making Scatter Module";
            ModuleInfoViewModels[0] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = T265ToLACCOffset.X, y = T265ToLACCOffset.Y, z = T265ToLACCOffset.Z },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        scatterGain,
                                                        scatterGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "MonoScatterLUT.csv"));

            Debug.WriteLine("Making Abosrober Module");
            VMStatus = "Making Absorber Module";
            ModuleInfoViewModels[8] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = T265ToLACCOffset.X, y = T265ToLACCOffset.Y, z = T265ToLACCOffset.Z - 0.25 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        absorberGain,
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

            VMStatus = "Initiate LACC Done";


        }

        public void InitiateSingleHeadQuadType()
        {
            try
            {
                IsLACCModuleInitiate = false;

                var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = false, Order = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

                double offset = 0.083;
                double[] xOffset = new double[] { -offset, +offset, -offset, +offset };
                double[] yOffset = new double[] { -offset, -offset, +offset, +offset };
                double offsetZ = -(0.251 + (31.5 - 21.5) / 1000);
                Debug.WriteLine("Making Scatter Module");
                VMStatus = "Making Scatter Module";
                for (int i = 0; i < 4; ++i)
                {
                    var scatterGain = LACC_Module.LoadGain(Path.Combine(LUTFolderDirectory, $"GainCorrectionMatrix_447278_scatter_{i + 1}.csv"));
                    var scatterMlpeGain = LACC_Module.LoadGain(Path.Combine(LUTFolderDirectory, $"GainCorrectionMatrix_447278_scatter_{i + 1}.csv"));
                    ModuleInfoViewModels[i] = new ModuleInfoViewModel(ModuleInfo.QuadSingleHead,
                                                new LACC_Module.ModuleOffset { x = T265ToLACCOffset.X + xOffset[i], y = T265ToLACCOffset.Y + yOffset[i], z = T265ToLACCOffset.Z },
                                                new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                scatterGain,
                                                scatterMlpeGain,
                                                pmtOrderInfo,
                                                Path.Combine(LUTFolderDirectory, $"LUT9chEXP_447278_{i + 1}_20210610_1mm_step2.csv"));
                }

                Debug.WriteLine("Making Abosrober Module");
                VMStatus = "Making Absorber Module";
                for (int i = 0; i < 4; ++i)
                {
                    var absorberGain = LACC_Module.LoadGain(Path.Combine(LUTFolderDirectory, $"EnergyGainCorrectionMatrix_absorber_{i + 1}.csv"));
                    ModuleInfoViewModels[i + 8] = new ModuleInfoViewModel(ModuleInfo.QuadSingleHead,
                                                            new LACC_Module.ModuleOffset { x = T265ToLACCOffset.X + xOffset[i], y = T265ToLACCOffset.Y + yOffset[i], z = T265ToLACCOffset.Z + offsetZ },
                                                            new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                            absorberGain,
                                                            absorberGain,
                                                            pmtOrderInfo,
                                                            Path.Combine(LUTFolderDirectory, $"LUT9chEXP_447279_{ i + 1 }_20210329_1mm_step2.csv"));
                }
                LACC_Control_Static = new LACC_Control(new LACC_Module[] { ModuleInfoViewModels[0].Module, ModuleInfoViewModels[1].Module, ModuleInfoViewModels[2].Module, ModuleInfoViewModels[3].Module },
                                                        new LACC_Module[] { ModuleInfoViewModels[8].Module, ModuleInfoViewModels[9].Module, ModuleInfoViewModels[10].Module, ModuleInfoViewModels[11].Module });
                IsLACCModuleInitiate = true;
                initiating = false;

                VMStatus = "Initiate LAHGI Quad Single head Success!";
            }
            catch
            {

                VMStatus = "Initiate LAHGI Quad Single head Failed";
            }


        }

        public void InitiateDualHeadQuadType()
        {
            IsLACCModuleInitiate = false;
        }

        public ObservableCollection<ModuleInfoViewModel> ModuleInfoViewModels { get; set; } //16 Channels

        private AsyncCommand initiateLACCommand;
        public IAsyncCommand InitiateLACCommand
        {
            get { return (this.initiateLACCommand) ?? (this.initiateLACCommand = new AsyncCommand(InitiateLACCAsync)); }
        }
        private bool initiating;
        private async Task InitiateLACCAsync()
        {
            if (initiating == true)
                return;


            initiating = true;
            IsLACCModuleInitiate = false;
            switch (this.selectedModuleInfo)
            {
                case ModuleInfo.Mono:
                    await Task.Run(()=>InitiateMonoType()).ConfigureAwait(false);
                    break;
                case ModuleInfo.QuadSingleHead:
                    await Task.Run(() => InitiateSingleHeadQuadType()).ConfigureAwait(false);
                    break;
                case ModuleInfo.QuadDualHead:
                    await Task.Run(() => InitiateDualHeadQuadType()).ConfigureAwait(false);
                    break;
            }
        }

        private AsyncCommand<string> ecalInfoChangedCommand;
        public ICommand EcalInfoChangedCommand
        {
            get { return (this.ecalInfoChangedCommand) ?? (this.ecalInfoChangedCommand = new AsyncCommand<string>(EcalInfoChanged)); }
        }
        private async Task EcalInfoChanged(string a)
        {
            Debug.WriteLine("Channel num is " + a);
            try
            {
                int channelNum = Convert.ToInt32(a);
                await Task.Run(()=>ModuleInfoViewModels[channelNum].Fitting()).ConfigureAwait(false);
            }
            catch
            {
                Debug.WriteLine("EcalInfoChanged parameter is wrong");
            }

        }

        #endregion

        #region Test Functions
        private AsyncCommand<object> testCommand;
        public ICommand TestCommand
        {
            get { return (this.testCommand) ?? (this.testCommand = new AsyncCommand<object>(TestFunction)); }
        }

        private async Task TestFunction(object t)
        {
            await Task.Run(() =>
            {
                while (false)
                {
                    
                    //Trace.WriteLine("DataInQueue count: " + FPGAControl.DataInQueue.Count);
                    //Trace.WriteLine("ParsedQueue count: " + FPGAControl.ParsedQueue.Count);                    
                    //Trace.WriteLine("ShortArrayQueue count: " + FPGAControl.ShortArrayQueue.Count);
                    //Thread.Sleep(1000);
                }
                
            });
            // Write Function to be used     
        }

        private string vmStatus;
        public string VMStatus
        {
            get { return vmStatus; }
            set { vmStatus = value; OnPropertyChanged(nameof(VMStatus)); }
        }
        #endregion

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
