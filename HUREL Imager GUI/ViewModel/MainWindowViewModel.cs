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
using System.Windows.Input;

namespace HUREL_Imager_GUI.ViewModel
{
    internal class MainWindowViewModel : ViewModelBase
    {
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
