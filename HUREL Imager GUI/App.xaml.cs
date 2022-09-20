﻿using System;
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

            //NativeMethods.AllocConsole();
            logger.Info("Start application");

            logger.Info("Console loaded");
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("NTcxMjAyQDMxMzkyZTM0MmUzMEw2eUs1OURYTGswSnNaZ3p5WjlIcWdPQTcrM2UxWEdSbWd6TW9iUnRlcjA9");


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
    }
    static class NativeMethods
    {
        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        public static extern bool AllocConsole();
    }

}