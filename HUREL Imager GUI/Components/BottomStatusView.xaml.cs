using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Automation.Peers;
using System.Windows.Automation.Provider;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace HUREL_Imager_GUI.Views
{
    /// <summary>
    /// Interaction logic for BottomStatusView.xaml
    /// </summary>
    public partial class BottomStatusView : UserControl
    {
        private bool isGridOpen = false;
        public BottomStatusView()
        {
            InitializeComponent();
            ButtonStatus.Click += OnButtonsClickEvent;
        }


        private void OnButtonsClickEvent(object? btn, EventArgs eventArgs)
        {
            if (isGridOpen == false)
            {
                ButtonAutomationPeer peer = new ButtonAutomationPeer(BtnOpen);
                IInvokeProvider? invokeProv = peer.GetPattern(PatternInterface.Invoke) as IInvokeProvider;
                invokeProv?.Invoke();    
                isGridOpen = true;
            }
            else
            {
                ButtonAutomationPeer peer = new ButtonAutomationPeer(BtnClose);
                IInvokeProvider? invokeProv = peer.GetPattern(PatternInterface.Invoke) as IInvokeProvider;
                invokeProv?.Invoke();
                isGridOpen = false;
            }
        }
    }
}
