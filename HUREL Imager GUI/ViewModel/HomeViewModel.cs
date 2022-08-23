using HUREL.Compton;
using log4net;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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

        private BitmapImage realtimeRGB;
        public BitmapImage RealtimeRGB
        {
            get { return realtimeRGB; }
            set
            {
                realtimeRGB = value;
                OnPropertyChanged(nameof(RealtimeRGB));
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

            TopButtonViewModel = new TopButtonViewModel();
            SpectrumViewModel = new SpectrumViewModel();
            ThreeDimensionalViewModel = new ThreeDimensionalViewModel();
            SourceDirectionViewModel = new SourceDirectionViewModel();

            TestValue = "Hello World";
            logger.Info("HomeViewModel Loaded");
            LoopTask = Task.Run(Loop);
        }

        private Task LoopTask;
        private bool RunLoop = true;
        private void Loop()
        {
            while(RunLoop)
            {
                BitmapImage? temp = LahgiApi.GetRgbImage();
                if (temp != null)
                {
                    RealtimeRGB = temp;
                }
                
            }
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
            if (LoopTask != null)
            {
                RunLoop = false;
                LoopTask.Wait();
            }
        }
    }
}
