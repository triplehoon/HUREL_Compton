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
using HUREL.Compton.RadioisotopeAnalysis;
using MathNet.Numerics;
using MathNet.Numerics.Statistics;
using Python.Runtime;

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



            ModuleEnergySpectrums = new List<ObservableCollection<HistoEnergy>>();

            for (int i = 0; i < 16; i++)
            {
                ModuleEnergySpectrums.Add(new ObservableCollection<HistoEnergy>());
            }

            InitiateAxis();


            InitiateLACCAsync().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            TestFunction("").SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
        }





        private ModuleInfo selectedModuleInfo = ModuleInfo.QuadDualHead;
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
            FPGAControl.SetVaribles(FPGAVariable);
            await FPGAControl.Dispose().ConfigureAwait(false);
            // rsControl.Dispose();
        }

        #region Draw Graph

        private void DataUpdate()
        {           
            while (IsSessionStart)
            {
                DrawSpectrum();
                Thread.Sleep(1000);
            }
        }

        public void DrawMLPEPositions()
        {
            if(IsMLPEOn == false)
            {
                return;
            }

            List<double[]> scatterXYZE = new List<double[]>();

            List<double[]> absorberXYZE = new List<double[]>();
            LahgiWrapper_Static.GetAbsoluteListModeData(ref scatterXYZE, ref absorberXYZE);


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
            SpectrumEnergyNasa espect = new SpectrumEnergyNasa(histoEnergys);

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


            ObservableCollection<IsotopeInfo> isotopeInfos = new ObservableCollection<IsotopeInfo>();
            List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(espect.FindPeaks(662, 40, 10, 10), 1, 662, 40, 10);

            foreach (Isotope iso in DetectedIso)
            {
                string energy = "";
                foreach (double e in iso.PeakEnergy)
                {
                    energy += e.ToString("0.");
                    energy += " ";
                }

                isotopeInfos.Add(new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, energy));
            }
            foreach (var newIso in isotopeInfos)
            {
                bool isSameFound = false;
                foreach (var oldIso in IsotopeInfos)
                {
                    if (newIso.Name == oldIso.Name)
                    {
                        isSameFound = true;
                        continue;
                    }
                }
                if (!isSameFound)
                {
                    IsotopeInfos.Add(newIso);
                }
            }
            OnPropertyChanged(nameof(IsotopeInfos));
            sw.Stop();
            Trace.WriteLine("Draw spectrum done");
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
                LahgiWrapper_Static = new LahgiWrapper(eModuleManagedType.QUAD_DUAL); 
            });

            VMStatus = "Initiate LACC Done";
            initiating = false;
            IsLACCModuleInitiate = true;
            return;
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
            var vc = new HelixToolkit.Wpf.SharpDX.Vector3Collection();
            var cc = new HelixToolkit.Wpf.SharpDX.Color4Collection();
            var values = new List<double>();
            TrueReconMaxValue = 50 * 50 + 50 * 50 + 100 * 100;
            ProjectionMaxValue = TrueReconMaxValue;

            await Task.Run(() => {
                for (int x = -50; x <= 50; x+=5)
                {
                    for (int y = -50; y <= 50; y+=5)
                    {
                        for (int z = 0; z <= 100; z+=5)
                        {
                            vc.Add(new SharpDX.Vector3(Convert.ToSingle(x) / 100, Convert.ToSingle(y) / 100, Convert.ToSingle(z) / 100));
                            double value = x * x + y * y + z * z;
                            cc.Add(ImageRecon.ColorScaleJet((float)value, (float)projectionMinValue, (float)projectionMaxValue));
                            values.Add(value);
                        }
                    }
                }
            });


            RealtimePositionInfo = new PositionInfos(vc, cc, values);
            UpdatePointCloudProjection();
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
