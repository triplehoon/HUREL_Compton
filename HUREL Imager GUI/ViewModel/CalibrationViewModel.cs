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
        private static readonly ILog logger = LogManager.GetLogger(typeof(CalibrationViewModel));
        public override void Unhandle()
        {
            _energyCalibrationViewModel.Unhandle();
        }
    }
}
