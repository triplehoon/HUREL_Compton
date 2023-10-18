using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;

namespace HUREL_Imager_GUI.ViewModel
{
    public class HomeViewModel :ViewModelBase
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(HomeViewModel));

        private TopButtonViewModel _topButtonViewModel;
        public TopButtonViewModel TopButtonViewModel
        {
            get 
            { 
                return _topButtonViewModel; 
            } 
            set
            {
                _topButtonViewModel = value;
                OnPropertyChanged(nameof(TopButtonViewModel));
            }
        }
        private SpectrumViewModel _spectrumViewModel;
        
        public SpectrumViewModel SpectrumViewModel
        {
            get { return _spectrumViewModel; }
            set { _spectrumViewModel = value; OnPropertyChanged(nameof(SpectrumViewModel)); }
        }

        private ThreeDimensionalViewModel _threeDimensionalViewModel;
        public ThreeDimensionalViewModel ThreeDimensionalViewModel
        {
            get
            {
                return _threeDimensionalViewModel;
            }
            set
            {
                _threeDimensionalViewModel = value;
                OnPropertyChanged(nameof(ThreeDimensionalViewModel));
            }
        }

        private SourceDirectionViewModel _sourceDirectionViewModel;
        public SourceDirectionViewModel SourceDirectionViewModel
        {
            get
            {
               return _sourceDirectionViewModel; 
            }
            set
            {
                _sourceDirectionViewModel = value;
                OnPropertyChanged(nameof(SourceDirectionViewModel));
            }
        }

        private DoseRateViewModel _doseRateViewModel;
        public DoseRateViewModel DoseRateViewModel
        {
            get
            {
                return _doseRateViewModel;
            }
            set
            {
                _doseRateViewModel = value;
                OnPropertyChanged(nameof(DoseRateViewModel));
            }
        }

        private ReconstructionImageViewModel _reconstructionImageViewModel;
        public ReconstructionImageViewModel ReconstructionImageViewModel
        {
            get
            {
                return _reconstructionImageViewModel;
            }
            set
            {
                _reconstructionImageViewModel = value;
                OnPropertyChanged(nameof(ReconstructionImageViewModel));
            }
        }
        

        public HomeViewModel()
        {
            // Will be not null!
            _topButtonViewModel = null!;
            _spectrumViewModel = null!;
            _testValue = null!;
            _threeDimensionalViewModel = null!;
            _sourceDirectionViewModel = null!;
            _doseRateViewModel= null!;

            TopButtonViewModel = new TopButtonViewModel();
            SpectrumViewModel = new SpectrumViewModel();
            ThreeDimensionalViewModel = new ThreeDimensionalViewModel();
            SourceDirectionViewModel = new SourceDirectionViewModel();
            DoseRateViewModel = new DoseRateViewModel();
            ReconstructionImageViewModel= new ReconstructionImageViewModel();

            TestValue = "Hello World";
            logger.Info("HomeViewModel Loaded");

            DoseRateViewModel.AlarmUpdate += AlarmUpdate;   //230920 sbkwon : DoseRate Alarm Event

            LahgiApi.StatusUpdate += StatusUpdate;
        }

        //230920 sbkwon : DoseRate Alarm Event
        private void AlarmUpdate(object? sender, EventArgs e)
        {
            //if (App.MainVM is not null)
            //    App.MainVM.AlramVisibility = 

            if (e is AlarmEventArgs)
            {
                AlarmEventArgs alarmEventArgs = (AlarmEventArgs)e;

                AlarmDisplay(alarmEventArgs.AlarmStatus);
                DoseRateViewModel.Alarm(alarmEventArgs.AlarmStatus);
            }
        }

        private void AlarmDisplay(enAlarm alarm)
        {
            if (App.MainVM is not null)
            {
                App.MainVM.AlramVisibility = System.Windows.Visibility.Collapsed;

                if (alarm == enAlarm.enAlarm3)
                {
                    App.MainVM.AlramColor = System.Windows.Media.Brushes.Yellow;
                    App.MainVM.AlramCycleTime = 1000;
                    App.MainVM.AlramVisibility = System.Windows.Visibility.Visible;
                    App.MainVM.AlarmContent = "Alarm 3";
                }
                if (alarm == enAlarm.enAlarm4)
                {
                    App.MainVM.AlramColor = System.Windows.Media.Brushes.Orange;
                    App.MainVM.AlramCycleTime = 800;
                    App.MainVM.AlramVisibility = System.Windows.Visibility.Visible;
                    App.MainVM.AlarmContent = "Alarm 4";
                }
                if (alarm == enAlarm.enAlarm5)
                {
                    App.MainVM.AlramColor = System.Windows.Media.Brushes.Red;
                    App.MainVM.AlramCycleTime = 500;
                    App.MainVM.AlramVisibility = System.Windows.Visibility.Visible;
                    App.MainVM.AlarmContent = "Alarm 5";
                }
            }
        }

        private string statusMsg = "test";
        public string StatusMsg
        {
            get { return statusMsg; }
            set { statusMsg = value; OnPropertyChanged(nameof(StatusMsg)); }
        }

        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }
            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Massage)
                {
                    StatusMsg = LahgiApi.StatusMsg;
                }
            }
            StatusUpdateMutex.ReleaseMutex();

        }

      

        private string _testValue;
        public string TestValue
        {
            get { return _testValue; }
            set { _testValue = value; OnPropertyChanged(nameof(TestValue)); }
        }

        public override void Unhandle()
        {
            SpectrumViewModel.Unhandle();
            TopButtonViewModel.Unhandle();
            DoseRateViewModel.Unhandle();
            ThreeDimensionalViewModel.Unhandle();
            ReconstructionImageViewModel.Unhandle();

            LahgiApi.StatusUpdate -= StatusUpdate;
        }
        
    }
}
