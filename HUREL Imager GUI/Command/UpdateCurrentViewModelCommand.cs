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
            if (parameter is ViewType)
            {
                _navigator.CurrentViewModel?.Unhandle();

                ViewType viewType = (ViewType)parameter;
                switch (viewType)
                {
                    case ViewType.HOME_VIEW:
                        _navigator.CurrentViewModel = new HomeViewModel();                        
                        break;
                    case ViewType.SETTING_VIEW:
                        _navigator.CurrentViewModel = new SettingViewModel();
                        break;
                }
            }
        }
    }
}