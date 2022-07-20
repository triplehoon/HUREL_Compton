using HUREL_Imager_GUI.ViewModel;
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
    /// TopButtonsView.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class TopButtonsView : UserControl
    {
        public TopButtonsView()
        {
            InitializeComponent();
       
            this.Loaded += new RoutedEventHandler(View1_Loaded);

        }

        void View1_Loaded(object sender, RoutedEventArgs e)
        {
            Window w = Window.GetWindow(SaveButton);
            if (null != w)
            {
                w.LocationChanged += SaveButtonMove;
            }
        }

        void SaveButtonMove(object? sender, EventArgs args)
        {
            var offset = SaveButtonPopup.HorizontalOffset;
            SaveButtonPopup.HorizontalOffset = offset + 1;
            SaveButtonPopup.HorizontalOffset = offset;

            TimeSetupButtonPopup.HorizontalOffset = offset + 1;
            TimeSetupButtonPopup.HorizontalOffset = offset;
        }

        
    }
}
