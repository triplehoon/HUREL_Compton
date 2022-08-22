using HUREL.Compton;
using HUREL_Imager_GUI.State.Navigator;
using log4net;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

        public override void Unhandle()
        {
            BottomStatusViewModel.Unhandle();
        }
    }
}
