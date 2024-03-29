﻿using System;
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
using HUREL.Compton.CZT;
using HUREL.Compton.LACC;
using MathNet.Numerics;
using MathNet.Numerics.Statistics;


namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel : INotifyPropertyChanged
    {

        public static LACC_Control LACC_Control_Static;
        public static LahgiWrapper LahgiWrapper_Static;

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

            if (SRE3021API.IsTCPOpen && SRE3021API.IsUDPOpen)
            {
                Trace.WriteLine("SRE3021 Loading Successs");
            }





            InitiateCZTAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            InitiateRealsenseAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            InitiateLACCAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));          
            TestFunction("").SafeFireAndForget(onException: ex => Debug.WriteLine(ex));

        }





        private ModuleInfo selectedModuleInfo = ModuleInfo.QuadSingleHead;
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
            SRE3021API.Close();
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

            List<double[]> scatterXYZE = new List<double[]>();

            List<double[]> absorberXYZE = new List<double[]>();
            LahgiWrapper_Static.GetRelativeListModeData(ref scatterXYZE, ref absorberXYZE);


            Stopwatch sw = new Stopwatch();
            sw.Start();
            var temp1 = new List<MlpePositionInfo>();
            foreach (var lmdata in absorberXYZE)
            {
                if (lmdata == null)
                {
                    continue;
                }
                temp1.Add(new MlpePositionInfo(lmdata[0], lmdata[1]));
                
            }
            AbsorberPositionData = temp1;
            //ScatterPositionData.Clear();
            var temp2 = new List<MlpePositionInfo>();

            foreach (var lmdata in scatterXYZE)
            {
                if (lmdata == null)
                {
                    continue;
                }
                temp2.Add(new MlpePositionInfo(lmdata[0], lmdata[1]));

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
            for (uint i = 0; i < 16; ++i)
            {
                await Task.Run(() =>{
                    LahgiWrapper_Static.ResetSpectrum(i);
                    CZTSpectrumEnergy.Reset();
                }) ;

            }
            
        }

        public List<ObservableCollection<HistoEnergy>> ModuleEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> SumEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> ScatterEnergySpectrums { get; set; }
        public ObservableCollection<HistoEnergy> AbsorberEnergySpectrums { get; set; }
        public void DrawSpectrum()
        {

            Stopwatch sw = new Stopwatch();
            sw.Start();

            if (IsLACCModuleInitiate == false)
            {
                return;
            }
            List<double[]> eCounts = new List<double[]>();
            List<HistoEnergy> histoEnergys = new List<HistoEnergy>();

            for (uint i = 0; i < 16; ++i)
            {
                eCounts = new List<double[]>();
                LahgiWrapper_Static.GetSpectrum(i, ref eCounts);

                histoEnergys = new List<HistoEnergy>();

                foreach(var eData in eCounts)
                {
                    histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
                }    

                ModuleEnergySpectrums[(int)i] = new ObservableCollection<HistoEnergy>(histoEnergys);
                OnPropertyChanged($"ModuleEnergySpectrums[{i}]");
            }

            eCounts = new List<double[]>();
            LahgiWrapper_Static.GetSumSpectrum(ref eCounts);
            histoEnergys = new List<HistoEnergy>();
            foreach (var eData in eCounts)
            {
                histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
            }
            SumEnergySpectrums = new ObservableCollection<HistoEnergy>(histoEnergys);
            OnPropertyChanged(nameof(SumEnergySpectrums));

            eCounts = new List<double[]>();
            LahgiWrapper_Static.GetAbsorberSumSpectrum(ref eCounts);
            histoEnergys = new List<HistoEnergy>();
            foreach (var eData in eCounts)
            {
                histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
            }
            AbsorberEnergySpectrums = new ObservableCollection<HistoEnergy>(histoEnergys);
            OnPropertyChanged(nameof(AbsorberEnergySpectrums));

            eCounts = new List<double[]>();
            LahgiWrapper_Static.GetScatterSumSpectrum(ref eCounts);
            histoEnergys = new List<HistoEnergy>();
            foreach (var eData in eCounts)
            {
                histoEnergys.Add(new HistoEnergy(eData[0], (int)eData[1]));
            }
            ScatterEnergySpectrums = new ObservableCollection<HistoEnergy>(histoEnergys);
            OnPropertyChanged(nameof(ScatterEnergySpectrums));


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

        #region Mono
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
            VMStatus = "Making Scatter Module";
            ModuleInfoViewModels[0] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = -T265ToLACCOffset.X, y = -T265ToLACCOffset.Y, z = -T265ToLACCOffset.Z },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        scatterGain,
                                                        scatterGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "MonoScatterLUT.csv"));

            Debug.WriteLine("Making Abosrober Module");
            VMStatus = "Making Absorber Module";
            ModuleInfoViewModels[8] = new ModuleInfoViewModel(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = -T265ToLACCOffset.X, y = -T265ToLACCOffset.Y, z = -T265ToLACCOffset.Z - 0.25 },
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
        #endregion
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
            VMStatus = "Initiating LAHGI";
            initiating = true;
            await Task.Run(()=>
            { 
                LahgiWrapper_Static = new LahgiWrapper(eModuleManagedType.QUAD); 
            });

            VMStatus = "Initiate LACC Done";
            initiating = false;
            IsLACCModuleInitiate = true;
            return;
            //initiating = true;
            //IsLACCModuleInitiate = false;
            //switch (this.selectedModuleInfo)
            //{
            //    case ModuleInfo.Mono:
            //        await Task.Run(() => InitiateMonoType()).ConfigureAwait(false);
            //        break;
            //    case ModuleInfo.QuadSingleHead:
            //        await Task.Run(() => InitiateSingleHeadQuadType()).ConfigureAwait(false);
            //        break;
            //    case ModuleInfo.QuadDualHead:
            //        await Task.Run(() => InitiateDualHeadQuadType()).ConfigureAwait(false);
            //        break;
            //}
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
            for (int i = 0; i < 144; ++i)
            await Task.Run(() =>
            {
                while (false)
                {
                //    if (!initiating)
                //    {
                //        //LahgiWrapper_Static.AddListModeData()
                //    }
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
