using HUREL_Imager_GUI.ViewModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace HUREL_Imager_GUI.State.Navigator
{
    public enum ViewType
    {
        HOME_VIEW,
        CALIBRATION_VIEW,
        SETTING_VIEW
    }
    internal interface INavigator
    {
        ViewModelBase? CurrentViewModel { get; set;   }
        ICommand UpdateCurrentViewModelCommand { get; }
    }
}
