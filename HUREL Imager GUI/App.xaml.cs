using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using HUREL.Compton;

namespace HUREL_Imager_GUI
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        public App()
        {
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("NTcxMjAyQDMxMzkyZTM0MmUzMEw2eUs1OURYTGswSnNaZ3p5WjlIcWdPQTcrM2UxWEdSbWd6TW9iUnRlcjA9");
            
        }

        protected override void OnStartup(StartupEventArgs e)
        {
            MainWindow = new MainWindow();
            MainWindow.Show();

            base.OnStartup(e);
        }
    }
}
