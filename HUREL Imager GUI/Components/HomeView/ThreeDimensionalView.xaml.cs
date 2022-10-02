using System;
using System.Collections.Generic;
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

namespace HUREL_Imager_GUI.Components
{
    /// <summary>
    /// Interaction logic for ThreeDimensionalView.xaml
    /// </summary>
    public partial class ThreeDimensionalView : UserControl
    {
        public ThreeDimensionalView()
        {
            InitializeComponent();
            this.Loaded += new RoutedEventHandler(View1_Loaded);
        }
        void View1_Loaded(object sender, RoutedEventArgs e)
        {
            Window w = Window.GetWindow(SetupButtonPopup);
            if (null != w)
            {
                w.LocationChanged += SaveButtonMove;
            }
        }

        void SaveButtonMove(object? sender, EventArgs args)
        {
            var offset = SetupButtonPopup.HorizontalOffset;
            SetupButtonPopup.HorizontalOffset = offset + 1;
            SetupButtonPopup.HorizontalOffset = offset;           
        }
    }
}
