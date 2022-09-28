using HUREL_Imager_GUI.State.Navigator;
using HUREL_Imager_GUI.ViewModel;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace HUREL_Imager_GUI.Command
{
    internal class UpdateCurrentViewModelCommand : ObservableObject, ICommand
    {
        public event EventHandler? CanExecuteChanged;

        private INavigator _navigator;

        public UpdateCurrentViewModelCommand(INavigator navigator)
        {
            _navigator = navigator;
        }

        public bool CanExecute(object? parameter)
        {
            return true;
        }

        public void Execute(object? parameter)
        {
            if (!IsVewModelsLoaded)
            {
                homeViewModel = new HomeViewModel();
                settingViewModel = new SettingViewModel();
                calibrationViewModel = new CalibrationViewModel();
                IsVewModelsLoaded = true;
            }
            if (parameter is ViewType)
            {
                //_navigator.CurrentViewModel?.Unhandle();

                ViewType viewType = (ViewType)parameter;
                switch (viewType)
                {
                    case ViewType.HOME_VIEW:
                        _navigator.CurrentViewModel = homeViewModel;
                        //OnPropertyChanged(nameof(_navigator.CurrentViewModel));
                        break;
                    case ViewType.SETTING_VIEW:
                        _navigator.CurrentViewModel = settingViewModel;
                        //OnPropertyChanged(nameof(_navigator.CurrentViewModel));
                        break;
                    case ViewType.CALIBRATION_VIEW:
                        _navigator.CurrentViewModel = calibrationViewModel;
                        break;
                }
            }
        }

        static bool IsVewModelsLoaded = false;
        static private HomeViewModel? homeViewModel;
        static private SettingViewModel? settingViewModel;
        static private CalibrationViewModel? calibrationViewModel;

    }
}