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
using HUREL.Compton;

namespace mono_LACC_MLPE
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();


            short[] adcData = ADCData(LACCMLPE.MLPEMode.Quad)

        }



        private short[] ADCData(LACCMLPE.MLPEMode mode)
        {
            short[] shortArray;
            Random random = new Random();
            if (mode == LACCMLPE.MLPEMode.Mono)
            {
                shortArray = new short[36];
                for(int i =0; i <36; i++)
                {
                    shortArray[i] = (short) random.Next(0,1000);
                }
            }
            else                
            {//Mode Quad
                shortArray = new short[9];
                for (int i = 0; i < 9; i++)
                {
                    shortArray[i] = (short)random.Next(0, 1000);
                }
            }

            return shortArray;
        }
    }
}
