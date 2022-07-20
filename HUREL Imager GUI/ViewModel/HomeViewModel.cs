using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    public class HomeViewModel :ViewModelBase
    {
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
        public HomeViewModel()
        {
            TopButtonViewModel = new TopButtonViewModel();
            SpectrumViewModel = new SpectrumViewModel();
            TestValue = "Hello World";
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
        }
    }
}
