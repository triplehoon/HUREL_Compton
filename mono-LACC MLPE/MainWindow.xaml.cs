using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
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
            

            Task.Run(() => Loading1());

            


        }

        public BlockingCollection<short[]> ShortBuffer = new BlockingCollection<short[]>();


        public void ReadLMDataCSV(string csvDirector)
        {
            FileStream fs = new FileStream(csvDirector, FileMode.Open);
            StreamReader sr = new StreamReader(fs, Encoding.UTF8, false);
            while(!sr.EndOfStream)
            {
                var line = sr.ReadLine();

                var shortArray = Array.ConvertAll(line.Split(','), short.Parse);
                ShortBuffer.Add(shortArray);
                Thread.Sleep(0);
            }
        }

        public bool IsAquiring;
        public void AddLACCControlData()
        {
            short[] item = { };
            int checkAdding = 0;
            while(IsAquiring)
            {
                while(ShortBuffer.TryTake(out item))
                {
                    LACC_Control.AddListModeData(item, Matrix3D.Identity);
                    checkAdding++;
                    if(checkAdding == 5000)
                    {
                        checkAdding = 0;
                        SpectrumData = LACC_Control.SpectrumEnergys;
                        UpdateSpectrum();
                        
                    }
                }
            }

        }

        public void Loading1()
        {
            var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = true, Order = new int[] { 0, 18, 1, 19, 2, 20, 11, 29, 12, 28, 9, 27, 3, 21, 4, 22, 5, 23, 14, 32, 13, 31, 12, 30, 6, 24, 7, 25, 8, 26, 17, 35, 16, 34, 15, 33 } };

            var scatterGain = new double[37]  { 0.229400822535143,
                                                0.194663785680398,
                                                0.184236289538727,
                                                0.328845970032704,
                                                0.388535540257414,
                                                0.182997191802852,
                                                0.256568926962897,
                                                0.261898167575063,
                                                0.200476805251476,
                                                0.107149915166777,
                                                0.258711656044783,
                                                0.317571489375082,
                                                0.153968089385871,
                                                0.175575768904058,
                                                0.175069094879092,
                                                0.111338944874489,
                                                0.126300762020813,
                                                0.310296386792488,
                                                0.271149150631817,
                                                0.110569664442744,
                                                0.109709278149893,
                                                0.312856624047890,
                                                0.135521237098968,
                                                0.193823476297495,
                                                0.147240054519398,
                                                0.199701252506580,
                                                0.222085079797251,
                                                0.186253277487988,
                                                0.163246220676073,
                                                0.363372707108992,
                                                0.451220095983549,
                                                0.294538914503081,
                                                0.234470528667482,
                                                0.330946205527829,
                                                0.201129108512092,
                                                0.399876618388626,
                                                -19.1899788970696 };

            var absorberGain = new double[37] { 0.609106629714565,
                                                0.408225309758093,
                                                0.461847452592639,
                                                0.420864773543207,
                                                0.406298442910974,
                                                0.556871972880209,
                                                0.427062526383404,
                                                0.529611054266539,
                                                0.385468424382990,
                                                0.248421318082802,
                                                0.399864947053825,
                                                0.425536517980407,
                                                0.339859200857057,
                                                0.398740664113444,
                                                0.464483368090175,
                                                0.403390895135249,
                                                0.298422129818660,
                                                0.553180476402401,
                                                0.642667635434905,
                                                0.358890089937244,
                                                0.464030776465580,
                                                0.445993103539891,
                                                0.273774321638299,
                                                0.214176752360862,
                                                0.621807100373737,
                                                0.356965167293123,
                                                0.376619470434398,
                                                0.289744640131841,
                                                0.369076302531657,
                                                0.674687609116932,
                                                0.639591093149570,
                                                0.556966464257456,
                                                0.651793451901132,
                                                0.363504215341530,
                                                0.662096134248347,
                                                0.599963606291628,
                                                -20.6402542760799 };

            Dispatcher.Invoke(() => SatusMessage.Text = "Making Scatter Module");
            Debug.WriteLine("Making Scatter Module");
            LACC_Module monoScatter = new LACC_Module(  ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffet { x = 0, y = 0, z = 0 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        scatterGain,
                                                        pmtOrderInfo,
                                                        "E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\SingleMLPETable.csv");

            Dispatcher.Invoke(() => SatusMessage.Text = "Making Abosrober Module");
            Debug.WriteLine("Making Abosrober Module");
            LACC_Module monoAbsorber = new LACC_Module(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffet { x = 0, y = 0, z = -150 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        absorberGain,
                                                        pmtOrderInfo,
                                                        "E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\SingleMLPETable.csv");

            LACC_Control = new LACC_Control(monoScatter, monoAbsorber);
       

            Dispatcher.Invoke(() => SatusMessage.Text = "Loading Done");
        }



        public List<double> SpectrumData = new List<double>();
        public void UpdateSpectrum()
        {
            Dispatcher.Invoke(() => CountTextBlock.Text = SpectrumData.Count.ToString());            
        }

        public Task ReadLine;
        public Task AddLACC;
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            IsAquiring = true;
            ReadLine = Task.Run(() => ReadLMDataCSV("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\Ecal_Scatter_Cs137.csv"));
            AddLACC = Task.Run(() => AddLACCControlData());
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            IsAquiring = false;
            ReadLine.Wait();
            AddLACC.Wait();
        }
    }
}
