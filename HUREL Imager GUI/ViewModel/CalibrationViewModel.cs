using log4net;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    class CalibrationViewModel : ViewModelBase
    {
        private EnergyCalibrationViewModel _energyCalibrationViewModel = new EnergyCalibrationViewModel();
        public EnergyCalibrationViewModel EnergyCalibrationViewModel 
        { 
            get 
            { 
                return _energyCalibrationViewModel; 
            }
            set 
            { 
                _energyCalibrationViewModel = value;
                OnPropertyChanged(nameof(EnergyCalibrationViewModel));
            }
        }

        private InteractionPointViewModel _interactionPointViewModel = new InteractionPointViewModel();
        public InteractionPointViewModel InteractionPointViewModel
        {
            get { return _interactionPointViewModel; }
            set { _interactionPointViewModel = value; OnPropertyChanged(nameof(InteractionPointViewModel)); }
        }

        private static readonly ILog logger = LogManager.GetLogger(typeof(CalibrationViewModel));
        public override void Unhandle()
        {
            _energyCalibrationViewModel.Unhandle();
        }
    }
}
