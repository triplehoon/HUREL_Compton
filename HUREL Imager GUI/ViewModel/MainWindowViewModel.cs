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

        public MainWindowViewModel()
        {
            Navigator.CurrentViewModel = new HomeViewModel();
            logger.Info("MainViewModel loaded");

        }

        public override void Unhandle()
        {
         
        }
    }
}
