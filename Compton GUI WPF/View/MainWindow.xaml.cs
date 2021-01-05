using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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
            Syncfusion.Licensing.SyncfusionLicenseProvider.RegisterLicense("Mzc0OTQ4QDMxMzgyZTM0MmUzMFRlU2dMemQrMXRGZW1iQk96NklnOVBWNnhScjVDckxTT2p1M0crRHpXcjA9");
            InitializeComponent();
        }



        private void SfDataGrid_PreviewMouseWheel_ScrollViewer1(object sender, MouseWheelEventArgs e)
        {
            ScrollViewer1.ScrollToVerticalOffset(ScrollViewer1.VerticalOffset - e.Delta);
        }

        private void ComboBox_TargetUpdated(object sender, DataTransferEventArgs e)
        {
            ((ComboBox)sender).SelectedIndex = 0;
            Debug.WriteLine("Combox changed");
        }
    }
}


