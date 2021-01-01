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
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using HUREL.Compton.LACC;

namespace mono_LACC_MLPE
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public static LACC_Control LACC_Control;
        public MainWindow()
        {
            InitializeComponent();

            LACC_Module monoScatter = new LACC_Module(  ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffet { x = 0, y = 0, z = 0 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        new double[36] {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
                                                                        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                                                        new LACC_Module.ModulePMTOrderInfo(),
                                                        "");


            LACC_Module monoAbsorber = new LACC_Module(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffet { x = 0, y = 0, z = -150 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        new double[36] {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
                                                                        1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                                                        new LACC_Module.ModulePMTOrderInfo(),
                                                        "");

            LACC_Control = new LACC_Control(monoScatter, monoAbsorber);

            LACC_Control.AddListModeData(new short[144], Matrix3D.Identity);

            var spect = LACC_Control.SpectrumEnergys;
        }


    }
}
