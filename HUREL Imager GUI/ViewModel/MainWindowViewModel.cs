using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL_Imager_GUI.State.Navigator;
using log4net;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;

namespace HUREL_Imager_GUI.ViewModel
{
    internal class MainWindowViewModel : ViewModelBase
    {
        Stopwatch sw = new Stopwatch();

        //231017 sbkwon : Alarm 문구
        private string _alarmContent = "";
        public string AlarmContent
        {
            get => _alarmContent;
            set
            {
                if (_alarmContent != value)
                {
                    _alarmContent = value;
                    OnPropertyChanged(nameof(AlarmContent));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 투명도
        private double _alarmOpacity = 0.0;
        public double AlramOpacity
        {
            get => _alarmOpacity;
            set
            {
                if (_alarmOpacity != value)
                {
                    _alarmOpacity = value;
                    OnPropertyChanged(nameof(AlramOpacity));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 표시 여부 (visible / Collapsed)
        private Visibility _alramVisiblity = Visibility.Collapsed;
        public Visibility AlramVisibility
        {
            get => _alramVisiblity;
            set
            {
                if (_alramVisiblity != value)
                {
                    if (value is Visibility.Visible)
                        sw.Restart();
                    else
                        sw.Stop();

                    _alramVisiblity = value;
                    OnPropertyChanged(nameof(AlramVisibility));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 배경 색
        private Brush _alramColor = Brushes.Yellow;
        public Brush AlramColor
        {
            get => _alramColor;
            set
            {
                if (_alramColor != value)
                {
                    _alramColor = value;
                    OnPropertyChanged(nameof(AlramColor));
                }
            }
        }

        //230921 sbkwon : Alarm 화면 깜빡임 주기
        private long _alarmCycleTime = 500;
        public long AlramCycleTime
        {
            get => _alarmCycleTime;
            set
            {
                if (value != _alarmCycleTime)
                {
                    _alarmCycleTime = value;
                    OnPropertyChanged(nameof(AlramCycleTime));
                }
            }
        }

        private readonly ILog logger = LogManager.GetLogger(nameof(MainWindowViewModel));

        public INavigator Navigator { get; set; } = new Navigator();

        private BottomSatusViewModel bottomStatusViewModel;
        public BottomSatusViewModel BottomStatusViewModel
        {
            get { return bottomStatusViewModel; }
            set
            {
                bottomStatusViewModel = value;
                OnPropertyChanged(nameof(BottomStatusViewModel));
            }
        }

        public MainWindowViewModel()
        {
            bottomStatusViewModel = null!;
            Navigator.UpdateCurrentViewModelCommand.Execute(ViewType.HOME_VIEW);
            logger.Info("MainViewModel loaded");
            BottomStatusViewModel = new BottomSatusViewModel();

            //230921 sbkwon : 화면 표시 - 주사율 단위 호출
            var alTurSt = false;
            CompositionTarget.Rendering += (sender, e) =>
            {
                if (AlramVisibility is Visibility.Visible)
                    if (sw.ElapsedMilliseconds >= AlramCycleTime)
                    {
                        sw.Restart();

                        alTurSt = !alTurSt;

                        AlramOpacity = alTurSt ? 0.5 : 0;
                    }
            };
        }

        private AsyncCommand? closingCommand = null;
        public ICommand ClosingCommand
        {
            get { return closingCommand ?? (closingCommand = new AsyncCommand(Closing)); }
        }
        private async Task Closing()
        {
            Navigator?.CurrentViewModel?.Unhandle();
            LahgiApi.StopAll();
            Unhandle();
        }

        public override void Unhandle()
        {
            BottomStatusViewModel.Unhandle();
        }
    }
}
