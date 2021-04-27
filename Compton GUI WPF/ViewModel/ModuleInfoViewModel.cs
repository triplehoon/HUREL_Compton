using HUREL.Compton.LACC;
using MathNet.Numerics;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using static HUREL.Compton.LACC.LACC_Module;

namespace Compton_GUI_WPF.ViewModel
{
    public class ModuleInfoViewModel : INotifyPropertyChanged
    {
        protected int moduleNum;
        public int ModuleNum
        {
            get { return moduleNum; }
            set { moduleNum = value; }
        }

        public LACC_Module Module { get; set; }


        protected double ecalEquationInfoA;
        public double EcalEquationInfoA
        {
            get { return ecalEquationInfoA; }
            set { ecalEquationInfoA = value; OnPropertyChanged(nameof(EcalEquationInfoA)); }
        }
        
        protected double ecalEquationInfoB;
        public double EcalEquationInfoB
        {
            get { return ecalEquationInfoB; }
            set { ecalEquationInfoB = value; OnPropertyChanged(nameof(EcalEquationInfoB)); }
        }
        
        protected double ecalEquationInfoC;
        public double EcalEquationInfoC
        {
            get { return ecalEquationInfoC; }
            set { ecalEquationInfoC = value; OnPropertyChanged(nameof(EcalEquationInfoC)); }
        }
        public void Fitting()
        {
            List<double> xValues = new List<double>();
            List<double> yValues = new List<double>();
            foreach (var info in EcalInfos)
            {
                xValues.Add(info.MeasuredEnergy);
                yValues.Add(info.TrueEnergy);
            }
            double[] xValuesArr = xValues.ToArray();
            double[] yValuesArr = yValues.ToArray();
            var fit = Fit.Polynomial(xValuesArr, yValuesArr, 2);
            EcalEquationInfoC = fit[0];
            EcalEquationInfoB = fit[1];
            EcalEquationInfoA = fit[2];
            Module.ModuleEcalData = new EcalVar { a = EcalEquationInfoA, b = EcalEquationInfoB, c = EcalEquationInfoC };
        }


        protected string lutCSVFileLink;
        public string LUTCSVFileLink
        {
            get { return lutCSVFileLink; }
            set
            {
                lutCSVFileLink = value;
                OnPropertyChanged(nameof(LUTCSVFileLink));
            }
        }
        public void LUTCSVFileChange()
        {
            try
            {
                Module = new LACC_Module(Module.SetupModuleInfo,
                                         Module.ModuleOffetData,
                                         Module.ModuleEcalData,
                                         Module.EnergyGain,
                                         Module.MLPEGain,
                                         Module.ModulePMTOrder,
                                         LUTCSVFileLink,
                                         ModuleNum);
            }
            catch
            {
                Debug.WriteLine("LACC Module Setting Failed");
            }
        }
        
        
        protected ObservableCollection<double> energyGain;
        public ObservableCollection<double> EnergyGain
        {
            get { return energyGain; }
            set 
            {
                energyGain = value;
                OnPropertyChanged(nameof(EnergyGain));
            }
        }
        public void SetEnergyGain()
        {
            Module.EnergyGain = EnergyGain.ToArray();
        }

        protected ObservableCollection<double> mlpeGain;
        public ObservableCollection<double> MLPEGain
        {
            get { return mlpeGain; }
            set
            {
                mlpeGain = value;
                OnPropertyChanged(nameof(MLPEGain));
            }
        }
        public void SetMLPEGain()
        {
            Module.MLPEGain = MLPEGain.ToArray();
        }


        public bool IsModuleSet { get; set; }

        public ModuleOffset Offset { get; set; }

        public ObservableCollection<EcalInfo> EcalInfos { get; }
        public ModuleInfoViewModel()
        {

        }
        public ModuleInfoViewModel(ModuleInfo mode, ModuleOffset offset, EcalVar ecalData, double[] egain, double[] mlpegain, ModulePMTOrderInfo pmtOrder, string csvFileLUT, int moduleNumber = 0)
        {
            Offset = offset;
            this.ModuleNum = moduleNumber;
            EcalInfos = new ObservableCollection<EcalInfo>();
            EcalInfos.Add(new EcalInfo(60, 60, "Am-241"));
            EcalInfos.Add(new EcalInfo(662, 662, "Cs-137"));
            EcalInfos.Add(new EcalInfo(511, 511, "Na-22"));
            EcalInfos.Add(new EcalInfo(1275, 1275, "Na-22"));
            EcalInfos.Add(new EcalInfo(1173, 1173, "Co-60"));
            EcalInfos.Add(new EcalInfo(1332, 1332, "Co-60"));
            EcalEquationInfoA = ecalData.a;
            EcalEquationInfoB = ecalData.b;
            EcalEquationInfoC = ecalData.c;
            LUTCSVFileLink = csvFileLUT;
            EnergyGain = new ObservableCollection<double>(egain);
            MLPEGain = new ObservableCollection<double>(mlpegain);
            try
            {
                Module = new LACC_Module(mode,
                                         offset,
                                         ecalData,
                                         egain,
                                         mlpegain,
                                         pmtOrder,
                                         csvFileLUT,
                                         moduleNum);
                IsModuleSet = true;
            }
            catch
            {
                IsModuleSet = false;
            }
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
            get
            {
                return trueEnergy;
            }
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
