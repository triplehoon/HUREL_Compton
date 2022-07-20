using HUREL.Compton;
using HUREL_Imager_GUI.State.Navigator;
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
        public INavigator Navigator { get; set; } = new Navigator();

        public MainWindowViewModel()
        {
            Navigator.CurrentViewModel = new HomeViewModel();
            Debug.WriteLine("MainWindowViewModel loaded");


        }

        public override void Unhandle()
        {
         
        }
    }
}
