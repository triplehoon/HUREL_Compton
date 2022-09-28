using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using System.Windows;
using HUREL.Compton;
using HUREL_Imager_GUI.ViewModel;
using log4net;
using log4net.Appender;

namespace HUREL_Imager_GUI
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application        
    {
        private static readonly ILog logger = LogManager.GetLogger(typeof(App));
        public App()
        {
            ShutdownMode = ShutdownMode.OnLastWindowClose;
            //NativeMethods.AllocConsole();
            logger.Info("Start application");
            
            logger.Info("Console loaded");
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("NTcxMjAyQDMxMzkyZTM0MmUzMEw2eUs1OURYTGswSnNaZ3p5WjlIcWdPQTcrM2UxWEdSbWd6TW9iUnRlcjA9");
            logger.Info("Config Setting");
            InitialLizeConfigFile();
        }
        protected override void OnStartup(StartupEventArgs e)
        {
          
            MainWindow = new MainWindow();
            MainWindow.DataContext = new MainWindowViewModel();
            MainWindow.Show();
            LahgiApi.InitiateLaghi();
            LahgiApi.InititateRtabmap();
            base.OnStartup(e);

        }

        private void InitialLizeConfigFile()
        {
            var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            var appSetting = configFile.AppSettings.Settings;
            if(appSetting["Test"] ==null)
            {
                appSetting.Add("Test", "0");
            }
            if (appSetting["ref_x"] == null)
            {
                appSetting.Add("ref_x", "662");
            }
            if (appSetting["ref_fwhm"] == null)
            {
                appSetting.Add("ref_fwhm", "50");
            }
            if (appSetting["ref_at_0"] == null)
            {
                appSetting.Add("ref_at_0", "10");
            }
            if (appSetting["min_snr"] == null)
            {
                appSetting.Add("min_snr", "5");
            }

            configFile.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
        }
    }
    static class NativeMethods
    {
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool AllocConsole();
    }

}
