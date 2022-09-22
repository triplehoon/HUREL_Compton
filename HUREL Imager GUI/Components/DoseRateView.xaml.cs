using Syncfusion.UI.Xaml.Gauges;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Policy;
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

namespace HUREL_Imager_GUI.Components
{
    /// <summary>
    /// Interaction logic for DoseRateView.xaml
    /// </summary>
    public partial class DoseRateView : UserControl
    {
        public DoseRateView()
        {
            InitializeComponent();
        }
        /// <summary>
        /// Dangerous Radiation Zone: >10 R/h (>0.1 Sv/h); inside this line dose rates can be higher
        ///Hot zone: >10 mR/h(>0.1 mSv/h); inside this line, dose rates can be higher
        ///Cold zone:  < 10 mR/h (<0.1 mSv/h); beyond this perimeter, dose rates will be low
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void scale_LabelCreated(object sender, LabelCreatedEventArgs e)
        {
            switch ((string)e.LabelText)
            {


                case "0":
                    e.LabelText = "0";
                    break;
                case "1":
                    e.LabelText = "0.1 µ";
                    break;
                case "2":
                    e.LabelText = "1 µ";
                    break;
                case "3":
                    e.LabelText = "10 µ";
                    break;
                case "4":
                    e.LabelText = "100 µ";
                    break;
                case "5":
                    e.LabelText = "1 m";
                    break;
                case "6":
                    e.LabelText = "10 m";
                    break;
                case "7":
                    e.LabelText = "100 m";
                    break;
                case "8":
                    e.LabelText = "1000 m";
                    break;
            }
        }

    }
}
