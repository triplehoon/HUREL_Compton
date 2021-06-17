using MahApps.Metro.Controls;
using Syncfusion.Windows.Shared;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace Compton_GUI_WPF
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            NativeMethods.AllocConsole();

            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("Mzc0OTQ4QDMxMzgyZTM0MmUzMFRlU2dMemQrMXRGZW1iQk96NklnOVBWNnhScjVDckxTT2p1M0crRHpXcjA9");
            try
            {
                InitializeComponent();
            }
            catch
            {

            }
            Debug.WriteLine("Initial Done");
            ComboboxSelectedSpectrum.ItemsSource = ComboboxSelectedSpectrumItems;
        }


        static class NativeMethods
        {
            [DllImport("kernel32.dll", SetLastError = true)]
            [return: MarshalAs(UnmanagedType.Bool)]
            public static extern bool AllocConsole();
        }

        private string[] ComboboxSelectedSpectrumItems = new string[]
        {
            "Scatter",
            "Absorber",
            "Sum",
            "Channel 0",
            "Channel 1",
            "Channel 2",
            "Channel 3",
            "Channel 4",
            "Channel 5",
            "Channel 6",
            "Channel 7",
            "Channel 8",
            "Channel 9",
            "Channel 10",
            "Channel 11",
            "Channel 12",
            "Channel 13",
            "Channel 14",
            "Channel 15",
            "CZT"
        };
      


        private void SfDataGrid_PreviewMouseWheel_ScrollViewer1(object sender, MouseWheelEventArgs e)
        {
            ScrollViewer1.ScrollToVerticalOffset(ScrollViewer1.VerticalOffset - e.Delta);
        }

        private void ComboBox_TargetUpdated(object sender, DataTransferEventArgs e)
        {
            ((ComboBox)sender).SelectedIndex = 0;
            Debug.WriteLine("Combox changed");
        }

        bool SfChart_MouseDown_Check = true;
        private void SfChart_MouseDown(object sender, MouseButtonEventArgs e)
        {
            
            Point position = new Point
            {
                X = e.GetPosition(SpectrumMain).X - SpectrumMain.SeriesClipRect.Left,
                Y = e.GetPosition(SpectrumMain).Y - SpectrumMain.SeriesClipRect.Top
            };
            //PointToValue converts window coordinates to chart X,Y coordinates
            double xValueDouble = SpectrumMain.PointToValue(this.SpectrumMain.PrimaryAxis, position);
            double yValue = SpectrumMain.PointToValue(this.SpectrumMain.SecondaryAxis, position);
            int xValue = Convert.ToInt32(xValueDouble);
            
            if (SfChart_MouseDown_Check)
            {
                TBMinE.Value = xValue;
                SfChart_MouseDown_Check = false;
            }
            else
            {
                TBMaxE.Value = xValue;
                SfChart_MouseDown_Check = true;
            }

            Debug.WriteLine("SfChart_MouseDown");
        }

        private void ComboboxSelectedSpectrum_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            Binding myBinding = new Binding();
            var channel = ((string)((System.Windows.Controls.ComboBox)sender).SelectedItem).Split(' ');
            if (channel[0] == "CZT")
            {

                myBinding = new Binding();
                myBinding.Path = new PropertyPath($"CZTSpectrum");
                myBinding.Mode = BindingMode.TwoWay;
                MainSpectrumView.SetBinding(Syncfusion.UI.Xaml.Charts.FastLineBitmapSeries.ItemsSourceProperty, myBinding);
                return;
            }

            
            if (channel[0] == "Absorber")
            {
                myBinding = new Binding();
                myBinding.Path = new PropertyPath($"AbsorberEnergySpectrums");
                myBinding.Mode = BindingMode.TwoWay;
                MainSpectrumView.SetBinding(Syncfusion.UI.Xaml.Charts.FastLineBitmapSeries.ItemsSourceProperty, myBinding);
                return;
            }

            if (channel[0] == "Sum")
            {
                myBinding = new Binding();
                myBinding.Path = new PropertyPath($"SumEnergySpectrums");
                myBinding.Mode = BindingMode.TwoWay;
                MainSpectrumView.SetBinding(Syncfusion.UI.Xaml.Charts.FastLineBitmapSeries.ItemsSourceProperty, myBinding);
                return;
            }

            if (channel[0] == "Scatter")
            {
                myBinding = new Binding();
                myBinding.Path = new PropertyPath($"ScatterEnergySpectrums");
                myBinding.Mode = BindingMode.TwoWay;
                MainSpectrumView.SetBinding(Syncfusion.UI.Xaml.Charts.FastLineBitmapSeries.ItemsSourceProperty, myBinding);
                return;
            }
            myBinding = new Binding();
            myBinding.Path = new PropertyPath($"ModuleEnergySpectrums[{channel[1]}]");
            myBinding.Mode = BindingMode.TwoWay;
            MainSpectrumView.SetBinding(Syncfusion.UI.Xaml.Charts.FastLineBitmapSeries.ItemsSourceProperty, myBinding);
        }
    }
}


