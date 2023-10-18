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
        internal static MainWindowViewModel? MainVM { get; private set; }   //230921 sbkwon : global variable

        private static readonly ILog logger = LogManager.GetLogger(typeof(App));
        public App()
        {
            ShutdownMode = ShutdownMode.OnLastWindowClose;
            //NativeMethods.AllocConsole();
            logger.Info("Start application");
            
            logger.Info("Console loaded");
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("NTcxMjAyQDMxMzkyZTM0MmUzMEw2eUs1OURYTGswSnNaZ3p5WjlIcWdPQTcrM2UxWEdSbWd6TW9iUnRlcjA9");
            logger.Info("Config Setting");
        }
        protected override void OnStartup(StartupEventArgs e)
        {
          
            MainWindow = new MainWindow();
            MainWindow.DataContext = new MainWindowViewModel();
            MainVM = MainWindow.DataContext as MainWindowViewModel;//230921 sbkwon : global variable
            MainWindow.Show();
            LahgiApi.InitiateLaghi();
            LahgiApi.InititateRtabmap();
            base.OnStartup(e);

        }

   
    }
    static class NativeMethods
    {
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool AllocConsole();
    }

}
