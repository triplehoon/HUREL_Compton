using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using HUREL.Compton.LACC;

namespace HUREL.Compton
{
    public class BinaryToLMData
    {

        public BlockingCollection<byte[]> ParsedQueue = new BlockingCollection<byte[]>();
        public BlockingCollection<short[]> ShortArrayQueue = new BlockingCollection<short[]>();

        public LACC_Control LACC;

        private string LUTFolderDirectory = Path.Combine(Directory.GetCurrentDirectory(), "LUT Files");

        public BinaryToLMData()
        {
            var pmtOrderInfo = new LACC_Module.ModulePMTOrderInfo { IsOrderChange = true, Order = new int[] { 0, 18, 1, 19, 2, 20, 11, 29, 10, 28, 9, 27, 3, 21, 4, 22, 5, 23, 14, 32, 13, 31, 12, 30, 6, 24, 7, 25, 8, 26, 17, 35, 16, 34, 15, 33 } };
            var scatterGain = new double[37]  {0.222287552011680,
                                                    0.208847009962622,
                                                    0.160835530297629,
                                                    0.350623925414967,
                                                    0.404254384165359,
                                                    0.173114587164014,
                                                    0.251973705604810,
                                                    0.287514444819041,
                                                    0.197784900587933,
                                                    0.113205828176507,
                                                    0.270750509096893,
                                                    0.324814715062910,
                                                    0.159132032020835,
                                                    0.179033913500545,
                                                    0.177842115156326,
                                                    0.108344401828327,
                                                    0.122989511991333,
                                                    0.333935336242705,
                                                    0.256168970319604,
                                                    0.115470302451087,
                                                    0.107632624571028,
                                                    0.343022471533058,
                                                    0.129540635721655,
                                                    0.184271389706723,
                                                    0.154867557833026,
                                                    0.183742374044755,
                                                    0.235758007303454,
                                                    0.183618330027555,
                                                    0.149858076110482,
                                                    0.404494624248347,
                                                    0.452139539299007,
                                                    0.304594382211978,
                                                    0.243182810827749,
                                                    0.343234564555548,
                                                    0.220940431055765,
                                                    0.370428100393800,
                                                    -19.3920305409253  };
            var absorberGain = new double[37] { 0.547118426,
                                                    0.423998687,
                                                    0.426206901,
                                                    0.408303161,
                                                    0.410912616,
                                                    0.557610406,
                                                    0.444274915,
                                                    0.513597437,
                                                    0.371204235,
                                                    0.279542663,
                                                    0.355811448,
                                                    0.405420482,
                                                    0.346096898,
                                                    0.358584417,
                                                    0.439085018,
                                                    0.381797553,
                                                    0.324406816,
                                                    0.486858039,
                                                    0.604246889,
                                                    0.357470801,
                                                    0.473510762,
                                                    0.437615232,
                                                    0.289740831,
                                                    0.200805523,
                                                    0.57044647 ,
                                                    0.383966989,
                                                    0.322863322,
                                                    0.3249445  ,
                                                    0.329417575,
                                                    0.743689349,
                                                    0.485597352,
                                                    0.733393991,
                                                    0.453444903,
                                                    0.433348959,
                                                    0.754890154,
                                                    0.5538563  ,
                                                    -0.430917509
                                                    };

            Debug.WriteLine("Making Scatter Module");
            
            LACC_Module scatterModule = new LACC_Module(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x =0, y = 0, z = 0 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        scatterGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "MonoScatterLUT.csv"));

            Debug.WriteLine("Making Abosrober Module");

            LACC_Module absorberModule = new LACC_Module(ModuleInfo.Mono,
                                                        new LACC_Module.ModuleOffset { x = 0, y =0, z = -0.250 },
                                                        new LACC_Module.EcalVar { a = 0, b = 1, c = 0 },
                                                        absorberGain,
                                                        pmtOrderInfo,
                                                        Path.Combine(LUTFolderDirectory, "MonoAbsorberLUT.csv"));            

            LACC = new LACC_Control(scatterModule, absorberModule);
        }

        public void LoadBinaryFile(string fileDirectory)
        {
            short flag = 0; //0 is nothing, 1 is find FE, 2 find second FE
            short countflag = 0;
            IsGenerateShortArrayBuff = true;
            IsAddingListModeData = true;

            byte[] dataBuffer = new byte[296];
            byte[] chk1 = new byte[296];
            byte[] chk2 = new byte[296];

            Task.Run(() => GenerateShortArrayBuffAsync());
            Task.Run(() => AddListModeData());
            byte[] bytes = File.ReadAllBytes(fileDirectory);


            foreach(byte b in bytes)
            {               
                if (flag == 2)
                {
                    dataBuffer[countflag] = b;
                    countflag++;
                    if (countflag == 296 && dataBuffer[294] == 0xFE && dataBuffer[295] == 0xFE)
                    {
                        chk1 = dataBuffer;
                        if (chk1 == chk2)
                        {
                            Debug.WriteLine("item is not changed");
                        }
                        chk2 = dataBuffer;
                        ParsedQueue.Add(dataBuffer);
                                             
                        dataBuffer = new byte[296];
                        countflag = 0;
                    }
                    else if (countflag == 296)
                    {
                        countflag = 0;
                        dataBuffer = new byte[296];
                    }
                }
                else
                {
                    if (b == 0xFE && flag == 0)
                    {
                        Debug.WriteLine("flag is 1");
                        flag = 1;
                    }
                    else if (b == 0xFE && flag == 1)
                    {
                        Debug.WriteLine("flag is 2");
                        flag = 2;
                    }
                    else
                    {
                        flag = 0;
                        Debug.WriteLine("flag is 0");
                    }
                }
            }

            IsGenerateShortArrayBuff = false;
            IsAddingListModeData = false;

        }



        private bool IsGenerateShortArrayBuff;
        private void GenerateShortArrayBuffAsync()
        {
            while (IsGenerateShortArrayBuff)
            {
                byte[] item;

                while (ParsedQueue.TryTake(out item))
                {
                    short[] shortArray = new short[144];
                    Buffer.BlockCopy(item, 0, shortArray, 0, 288);
                    ShortArrayQueue.Add(shortArray);
                }
            }
        }
        private bool IsAddingListModeData;
        private void AddListModeData()
        {
            short[] check1;
            short[] check2 = new short[256];
            LACC.ResetLMData();
            while (IsAddingListModeData)
            {
                short[] item;
                while (ShortArrayQueue.TryTake(out item))
                {
                    check1 = item;
                    if (check1 == check2)
                    {
                        Debug.WriteLine("CEHK");
                    }

                    Matrix3D matrix3D = Matrix3D.Identity;
                    check2 = item;
                    LACC.AddListModeData(item, matrix3D, true, 600, 720);
                }
            }
            Debug.WriteLine("Add List Mode Data Done");
        }






    }
}
