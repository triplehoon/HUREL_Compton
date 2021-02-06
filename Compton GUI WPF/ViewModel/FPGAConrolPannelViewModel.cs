using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Text.RegularExpressions;
using System.Windows.Data;
using System.Windows.Input;
using GalaSoft.MvvmLight.Command;
using GalaSoft.MvvmLight.Messaging;
using HUREL.Compton;


namespace Compton_GUI_WPF.ViewModel
{
    class FPGAConrolPannelViewModel : INotifyPropertyChanged
    {

        public FPGAConrolPannelViewModel()
        {
            
        }
        
        #region Varibles Set
        private string smoothWindowSamples;
        public string SmoothWindowSamples
        {
            get 
            {
                smoothWindowSamples = MainViewModel.FPGAVariable.SmoothWindow0x13.ToString();
                return smoothWindowSamples; 
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.SmoothWindow0x13 = Convert.ToInt32(value);
                    smoothWindowSamples = value;
                }
                catch
                {
                    SmoothWindowSamples = "0";
                }
            }
        }

        private string blMeasInterval;
        public string BlMeasInterval
        {
            get
            {
                blMeasInterval = MainViewModel.FPGAVariable.BaseLineInterval0x14.ToString();
                return blMeasInterval;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.BaseLineInterval0x14 = Convert.ToInt32(value);
                    blMeasInterval = value;
                }
                catch
                {
                    BlMeasInterval = "0";
                }
            }
        }

        private string blOffset;
        public string BlOffset
        {
            get
            {
                blOffset = MainViewModel.FPGAVariable.BaseLineOffset0x15.ToString();
                return blOffset;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.BaseLineOffset0x15 = Convert.ToInt32(value);
                    blOffset = value;
                }
                catch
                {
                    BlOffset = "0";
                }
            }
        }

        private string maxOffset;
        public string MaxOffset
        {
            get
            {
                maxOffset = MainViewModel.FPGAVariable.MaxOffset0x16.ToString();
                return maxOffset;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.MaxOffset0x16 = Convert.ToInt32(value);
                    maxOffset = value;
                }
                catch
                {
                    MaxOffset = "0";
                }
            }
        }

        private string maxMeasInterval;
        public string MaxMeasInterval
        {
            get
            {
                maxMeasInterval = MainViewModel.FPGAVariable.MaxMeasInterval0x17.ToString();
                return maxMeasInterval;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.MaxMeasInterval0x17 = Convert.ToInt32(value);
                    maxMeasInterval = value;
                }
                catch
                {
                    MaxMeasInterval = "0";
                }
            }
        }

        private string baselineTrigger_0;
        public string BaselineTrigger_0
        {
            get
            {
                baselineTrigger_0 = MainViewModel.FPGAVariable.TriggerThreshold0x19[0].ToString();
                return baselineTrigger_0;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[0] = Convert.ToInt32(value);
                    baselineTrigger_0 = value;
                }
                catch
                {
                    BaselineTrigger_0 = "0";
                }
            }
        }
        private string baselineTrigger_1;
        public string BaselineTrigger_1
        {
            get
            {
                baselineTrigger_1 = MainViewModel.FPGAVariable.TriggerThreshold0x19[1].ToString();
                return baselineTrigger_1;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[1] = Convert.ToInt32(value);
                    baselineTrigger_1 = value;
                }
                catch
                {
                    BaselineTrigger_1 = "0";
                }
            }
        }
        private string baselineTrigger_2;
        public string BaselineTrigger_2
        {
            get
            {
                baselineTrigger_2 = MainViewModel.FPGAVariable.TriggerThreshold0x19[2].ToString();
                return baselineTrigger_2;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[2] = Convert.ToInt32(value);
                    baselineTrigger_2 = value;
                }
                catch
                {
                    BaselineTrigger_2 = "0";
                }
            }
        }
        private string baselineTrigger_3;
        public string BaselineTrigger_3
        {
            get
            {
                baselineTrigger_3 = MainViewModel.FPGAVariable.TriggerThreshold0x19[3].ToString();
                return baselineTrigger_3;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[3] = Convert.ToInt32(value);
                    baselineTrigger_3 = value;
                }
                catch
                {
                    BaselineTrigger_3 = "0";
                }
            }
        }
        private string baselineTrigger_4;
        public string BaselineTrigger_4
        {
            get
            {
                baselineTrigger_4 = MainViewModel.FPGAVariable.TriggerThreshold0x19[4].ToString();
                return baselineTrigger_4;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[4] = Convert.ToInt32(value);
                    baselineTrigger_4 = value;
                }
                catch
                {
                    BaselineTrigger_4 = "0";
                }
            }
        }
        private string baselineTrigger_5;
        public string BaselineTrigger_5
        {
            get
            {
                baselineTrigger_5 = MainViewModel.FPGAVariable.TriggerThreshold0x19[5].ToString();
                return baselineTrigger_5;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[5] = Convert.ToInt32(value);
                    baselineTrigger_5 = value;
                }
                catch
                {
                    BaselineTrigger_5 = "0";
                }
            }
        }
        private string baselineTrigger_6;
        public string BaselineTrigger_6
        {
            get
            {
                baselineTrigger_6 = MainViewModel.FPGAVariable.TriggerThreshold0x19[6].ToString();
                return baselineTrigger_6;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[6] = Convert.ToInt32(value);
                    baselineTrigger_6 = value;
                }
                catch
                {
                    BaselineTrigger_6 = "0";
                }
            }
        }
        private string baselineTrigger_7;
        public string BaselineTrigger_7
        {
            get
            {
                baselineTrigger_7 = MainViewModel.FPGAVariable.TriggerThreshold0x19[7].ToString();
                return baselineTrigger_7;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[7] = Convert.ToInt32(value);
                    baselineTrigger_7 = value;
                }
                catch
                {
                    BaselineTrigger_7 = "0";
                }
            }
        }
        private string baselineTrigger_8;
        public string BaselineTrigger_8
        {
            get
            {
                baselineTrigger_8 = MainViewModel.FPGAVariable.TriggerThreshold0x19[8].ToString();
                return baselineTrigger_8;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[8] = Convert.ToInt32(value);
                    baselineTrigger_8 = value;
                }
                catch
                {
                    BaselineTrigger_8 = "0";
                }
            }
        }
        private string baselineTrigger_9;
        public string BaselineTrigger_9
        {
            get
            {
                baselineTrigger_9 = MainViewModel.FPGAVariable.TriggerThreshold0x19[9].ToString();
                return baselineTrigger_9;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[9] = Convert.ToInt32(value);
                    baselineTrigger_9 = value;
                }
                catch
                {
                    BaselineTrigger_9 = "0";
                }
            }
        }
        private string baselineTrigger_10;
        public string BaselineTrigger_10
        {
            get
            {
                baselineTrigger_10 = MainViewModel.FPGAVariable.TriggerThreshold0x19[10].ToString();
                return baselineTrigger_10;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[10] = Convert.ToInt32(value);
                    baselineTrigger_10 = value;
                }
                catch
                {
                    BaselineTrigger_10 = "0";
                }
            }
        }
        private string baselineTrigger_11;
        public string BaselineTrigger_11
        {
            get
            {
                baselineTrigger_11 = MainViewModel.FPGAVariable.TriggerThreshold0x19[11].ToString();
                return baselineTrigger_11;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[11] = Convert.ToInt32(value);
                    baselineTrigger_11 = value;
                }
                catch
                {
                    BaselineTrigger_11 = "0";
                }
            }
        }
        private string baselineTrigger_12;
        public string BaselineTrigger_12
        {
            get
            {
                baselineTrigger_12 = MainViewModel.FPGAVariable.TriggerThreshold0x19[12].ToString();
                return baselineTrigger_12;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[12] = Convert.ToInt32(value);
                    baselineTrigger_12 = value;
                }
                catch
                {
                    BaselineTrigger_12 = "0";
                }
            }
        }
        private string baselineTrigger_13;
        public string BaselineTrigger_13
        {
            get
            {
                baselineTrigger_13 = MainViewModel.FPGAVariable.TriggerThreshold0x19[13].ToString();
                return baselineTrigger_13;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[13] = Convert.ToInt32(value);
                    baselineTrigger_13 = value;
                }
                catch
                {
                    BaselineTrigger_13 = "0";
                }
            }
        }
        private string baselineTrigger_14;
        public string BaselineTrigger_14
        {
            get
            {
                baselineTrigger_14 = MainViewModel.FPGAVariable.TriggerThreshold0x19[14].ToString();
                return baselineTrigger_14;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[14] = Convert.ToInt32(value);
                    baselineTrigger_14 = value;
                }
                catch
                {
                    BaselineTrigger_14 = "0";
                }
            }
        }
        private string baselineTrigger_15;
        public string BaselineTrigger_15
        {
            get
            {
                baselineTrigger_15 = MainViewModel.FPGAVariable.TriggerThreshold0x19[15].ToString();
                return baselineTrigger_15;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerThreshold0x19[15] = Convert.ToInt32(value);
                    baselineTrigger_15 = value;
                }
                catch
                {
                    BaselineTrigger_15 = "0";
                }
            }
        }
 

        private string trgSlopePoint;
        public string TrgSlopePoint
        {
            get
            {
                trgSlopePoint = MainViewModel.FPGAVariable.TriggerSlopePoints0x18.ToString();
                return trgSlopePoint;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TriggerSlopePoints0x18 = Convert.ToInt32(value);
                    trgSlopePoint = value;
                }
                catch
                {
                    TrgSlopePoint = "0";
                }
            }
        }
        
        private string noTrgforDetTrg;
        public string NoTrgforDetTrg
        {
            get
            {
                noTrgforDetTrg = MainViewModel.FPGAVariable.NoTrgforDetTrg0x1a.ToString();
                return noTrgforDetTrg;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.NoTrgforDetTrg0x1a = Convert.ToInt32(value);
                    noTrgforDetTrg = value;
                }
                catch
                {
                    NoTrgforDetTrg = "0";
                }
            }
        }
        
        private string twForDetTrg;
        public string TWForDetTrg
        {
            get
            {
                twForDetTrg = MainViewModel.FPGAVariable.TWforDetTrg0x1b.ToString();
                return twForDetTrg;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.TWforDetTrg0x1b = Convert.ToInt32(value);
                    twForDetTrg = value;
                }
                catch
                {
                    TWForDetTrg = "0";
                }

            }
        }
        
        private string twForGlobalTrg;
        public string TWForGlobalTrg
        {
            get
            {
                twForGlobalTrg = MainViewModel.FPGAVariable.GlobalTriggerWindow0x12.ToString();
                return twForGlobalTrg;
            }
            set
            {
                try
                {
                    MainViewModel.FPGAVariable.GlobalTriggerWindow0x12 = Convert.ToInt32(value);
                    twForGlobalTrg = value;
                }
                catch
                {
                    TWForGlobalTrg = "0";
                }
            }
        }



        private bool[] measurementMode = new bool[] { true, false, false, false };        
        public bool[] MeasurementMode
        {
            get
            {               
                MainViewModel.FPGAVariable.CurrentMeasurementMode0x11 = (CRUXELLLACC.MeasurementMode)Array.IndexOf(measurementMode, true);
                Debug.WriteLine(MainViewModel.FPGAVariable.CurrentMeasurementMode0x11.ToString());
                return this.measurementMode;               
            }           
        }

        private CRUXELLLACC.MeasurementMode currentMode;
        public CRUXELLLACC.MeasurementMode CurrentMode
        {
            get {
                currentMode = MainViewModel.FPGAVariable.CurrentMeasurementMode0x11;
                return currentMode; 
            }
            set 
            {
                MainViewModel.FPGAVariable.CurrentMeasurementMode0x11 = value;
            }
        }



        private string debugText;
        public string DebugText
        {
            get { return debugText; }
            set { debugText = value; OnPropertyChanged(nameof(DebugText)); }
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

    public class MeasurementModeComparisonConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return value?.Equals((CRUXELLLACC.MeasurementMode)System.Convert.ToInt32(parameter));
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return value?.Equals(true) == true ? parameter : Binding.DoNothing;
        }
    }

    public class WindowStateMessage
    {
        public bool state;
    }

}
