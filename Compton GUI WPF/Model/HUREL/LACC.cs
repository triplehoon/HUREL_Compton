using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace HUREL.Compton.LACC
{

    /// <summary>
    /// Module type information. QuadSingleHead, QuadDualHead,Mono
    /// </summary>
    public enum ModuleInfo
    {
        Mono,
        QuadSingleHead,
        QuadDualHead
    }

    public record ChannelEnergy(int Channel, double Energy);
    public class LACC_Control
    {
        public ModuleInfo Module;
        /// <summary>
        /// FPGA Channel 0 to 7. For mono index 0 is Channel 0 to 3
        /// </summary>
        public LACC_Module[/*Channel Numer*/] LACC_Scatter_Modules;
        /// <summary>
        /// FPGA Channel 8 to 15. For mono index 0 is Channel 8 to 11
        /// </summary>
        public LACC_Module[/*Channel Numer*/] LACC_Absorber_Modules; 
        public List<LMData> ListedLMData = new List<LMData>();
        public List<ChannelEnergy> EnergySpect = new List<ChannelEnergy>();
        

        /// <summary>
        /// Quad type setting
        /// </summary>
        /// <param name="scatters"></param>
        /// <param name="absorbers"></param>
        /// <param name="module"></param>
        public LACC_Control(LACC_Module[] scatters, LACC_Module[] absorbers, ModuleInfo module = ModuleInfo.QuadSingleHead)
        {
            if (scatters.Length != 4 && scatters.Length != 8)
            {
                throw new ArgumentException();
            }

            if (absorbers.Length != 4 && absorbers.Length != 8)
            {
                throw new ArgumentException();
            }
            LACC_Scatter_Modules = scatters;
            LACC_Absorber_Modules = absorbers;
            Module = module;
        }

        /// <summary>
        /// Mono type setting
        /// </summary>
        /// <param name="scatter"></param>
        /// <param name="absorber"></param>
        /// <param name="module"></param>
        public LACC_Control(LACC_Module scatter, LACC_Module absorber, ModuleInfo module = ModuleInfo.Mono)
        {
            Module = ModuleInfo.Mono;            
            LACC_Scatter_Modules = new LACC_Module[1] { scatter };
            LACC_Absorber_Modules = new LACC_Module[1] { absorber };
        }

        public void AddListModeData(short[] fullADCArrayValue,Matrix3D deviceTransformation,bool isMLPEOn = false, double minE =0, double maxE=10000)
        {

            LMData lmData;
            if (Module == ModuleInfo.Mono)
            {
                short[] scatter = fullADCArrayValue[0..36];
                short[] absorber = fullADCArrayValue[72..108];

                int scatterProd = 1;
                foreach(short s in scatter)
                {
                    scatterProd *= s;
                    if (scatterProd == 0)
                        break;
                    scatterProd = 1;
                }

                int absorberProd = 1;
                foreach (short s in absorber)
                {
                    absorberProd *= s;
                    if (absorberProd == 0)
                        break;
                    absorberProd = 1;
                }

                if(scatterProd != 0 && absorberProd != 0)
                {
                    double scatterEcal = LACC_Scatter_Modules[0].GetEcal(scatter);
                    double absorberEcal = LACC_Absorber_Modules[0].GetEcal(absorber);

                    double combinedEcal = scatterEcal + absorberEcal;                    
                    EnergySpect.Add(new ChannelEnergy(0,scatterEcal));
                    EnergySpect.Add(new ChannelEnergy(8, absorberEcal));

                    if (combinedEcal > minE && combinedEcal < maxE && isMLPEOn)
                    {
                        var scatterMLPEdata = LACC_Scatter_Modules[0].FastPosAndEEstimate(scatter);
                        var absorberMLPEdata = LACC_Absorber_Modules[0].FastPosAndEEstimate(absorber);
                        lmData = new LMData(scatterMLPEdata.Item1, absorberMLPEdata.Item1, scatterMLPEdata.Item2, absorberMLPEdata.Item2, deviceTransformation);
                        ListedLMData.Add(lmData);
                    }                    
                }
                
                else if(scatterProd != 0 && absorberProd ==0)
                {
                    double scatterEcal = LACC_Scatter_Modules[0].GetEcal(scatter);
                    EnergySpect.Add(new ChannelEnergy(0, scatterEcal));

                    if (scatterEcal > minE && scatterEcal < maxE && isMLPEOn)
                    {
                        var scatterMLPEdata = LACC_Scatter_Modules[0].FastPosAndEEstimate(scatter);                       
                        lmData = new LMData(scatterMLPEdata.Item1, scatterMLPEdata.Item2, deviceTransformation);
                        ListedLMData.Add(lmData);
                    }
                }
                
                else if(scatterProd == 0 && absorberProd != 0)
                {
                    double absorberEcal = LACC_Absorber_Modules[0].GetEcal(absorber);
                    EnergySpect.Add(new ChannelEnergy(8, absorberEcal));
                }
                else
                {
                    //Do nothing
                }
            }
            else if (Module == ModuleInfo.QuadSingleHead)
            {
                throw new NotImplementedException();
            }
            else //(Module == ModuleInfo.QuadDualHead)
            {
               throw new NotImplementedException();
            }
        }

        public void ResetLMData()
        {
            ListedLMData = new List<LMData>();
            EnergySpect = new List<ChannelEnergy>();
            LMData.Reset();
        }

    }




    public class LACC_Module
    {
        public int ChannelNumber;

        public ModuleInfo SetupModuleInfo { get; init; }
        private int PmtCount;

        /// <summary>
        /// ax^2+bx+c
        /// </summary>
        public struct EcalVar
        {
            public double a;
            public double b;
            public double c;
        }
        
        public EcalVar ModuleEcalData { get; set; }
        /// <summary>
        /// x, y, z offset in [mm]
        /// </summary>
        public struct ModuleOffset
        {
            public double x;
            public double y;
            public double z;
        }

        public ModuleOffset ModuleOffetData { get; set; }
        private double ModuleOffsetX;
        private double ModuleOffsetY;
        private double ModuleOffsetZ;


        public double[] ModuleGain { get; set; }
        public ModulePMTOrderInfo ModulePMTOrder { get; init; }
        public class ModulePMTOrderInfo
        {
            public int[] Order = new int[] { 0, 1, 2, 3, 4, 5, 6, 7, 8,
                                             9,10,11,12,13,14,15,16,17,
                                            18,19,20,21,22,23,24,25,26,
                                            27,28,29,30,31,32,33,34,35};
            public bool IsOrderChange = false;
        }

        private List<MLPELUTElement> LUT = new List<MLPELUTElement>();
        private int LUTSize = 0;

        /// <summary>
        /// Look up table element structure
        /// </summary>
        private struct MLPELUTElement
        {
            public double Xpos;
            public double Ypos;
            public double Zpos;

            public List<double> LogMu;
            public double SumMu;
        }
        private double[] XposArray;
        private double[] YposArray;
        private double[] ZposArray;
        private double[][] LogMue;
        private double[] SumMu;



        /// </summary>
        /// <param name="mode"> QuadSingleHead, QuadDualHead Mono</param>
        /// <param name="offset"> offset. [mm]</param>
        /// <param name="ecalData">ax^2+bx+c</param>
        /// <param name="gain"> Scintilator check. </param>    
        /// <param name="channelNumber"> Channel check. </param>    
        /// <param name="csvFileLUT"> cvsFile link </param>    
        public LACC_Module(ModuleInfo mode, ModuleOffset offset, EcalVar ecalData, double[] gain, ModulePMTOrderInfo pmtOrder, string csvFileLUT, int channelNumber = 0)
        {
            SetupModuleInfo = mode;
            ChannelNumber = channelNumber;
            ModuleOffetData = offset;
            ModuleOffsetX = ModuleOffetData.x;
            ModuleOffsetY = ModuleOffetData.y;
            ModuleOffsetZ = ModuleOffetData.z;
            ModuleEcalData = ecalData;

            ModuleGain = gain;
            ModulePMTOrder = pmtOrder;

            loadLUT(csvFileLUT);
            MuArrays();
        }

        /// <summary>
        /// Loading Look Up Table for selected scintalator
        /// </summary>
        /// <param name="FileName"></param>
        private void loadLUT(string FileName)
        {
            StreamReader IO = new StreamReader(FileName);
            if (SetupModuleInfo == ModuleInfo.Mono)
            {
                PmtCount = 36;
            }
            else
            {
                PmtCount = 9;
            }

            try
            {
                while (!IO.EndOfStream)
                {
                    var line = IO.ReadLine();
                    double[] value = Array.ConvertAll(line.Split(','), Double.Parse);
                    var test = value[3..(int)(PmtCount + 3)];
                    MLPELUTElement element = new MLPELUTElement { Xpos = value[0], Ypos = value[1], Zpos = value[2], LogMu = new List<double>(value[3..(PmtCount + 3)]), SumMu = value[value.Length - 1] };
                    LUT.Add(element);
                }
                LUTSize = LUT.Count();


                ///LUT List to Array -> to make it Fast!!!!
                XposArray = new double[LUT.Count()];
                YposArray = new double[LUT.Count()];
                ZposArray = new double[LUT.Count()];
                LogMue = new double[LUT.Count()][];
                SumMu = new double[LUT.Count()];
                int i = 0;
                foreach (var lut in LUT)
                {
                    XposArray[i] = lut.Xpos; //mm to meter
                    YposArray[i] = lut.Ypos;
                    ZposArray[i] = lut.Zpos;
                    LogMue[i] = new double[PmtCount];
                    for (int j = 0; j < PmtCount; j++)
                    {
                        LogMue[i][j] = lut.LogMu[j];
                    }
                    SumMu[i] = lut.SumMu;
                    i++;
                }





                Debug.WriteLine("LUT Done!");
            }
            catch
            {
                Debug.WriteLine("Reading LUT Error");
            }
            finally
            {
                IO.Close();
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pmtADCValue">PMT Value in short array </param>
        /// <returns> {xpos,ypos,zpos,Energy}[mm,mm,mm,keV]</returns>
        public (Point3D, double) PosAndEEstimate(short[] pmtADCValue)
        {
            //Stopwatch sw = new Stopwatch();
            //sw.Start();

            Point3D point = new Point3D();
            double eCalEnergy = GetEcal(pmtADCValue);
            double max = 0;
            double maxChk = -50000;
            double val = 0;
            double[] pos = new double[3];
            double[] normalizePMTValue = new double[PmtCount];



            //sw.Stop();
            //Debug.WriteLine("0: " + sw.ElapsedTicks);
            //sw.Restart();
            if (ModulePMTOrder.IsOrderChange)
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    if (pmtADCValue[ModulePMTOrder.Order[i]] * ModuleGain[i] > max)
                    {
                        max = pmtADCValue[ModulePMTOrder.Order[i]] * ModuleGain[i];
                    }
                }
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ((pmtADCValue[ModulePMTOrder.Order[i]] * ModuleGain[i]) / max);
                }
            }
            else
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    if (pmtADCValue[i] * ModuleGain[i] > max)
                    {
                        max = pmtADCValue[i] * ModuleGain[i];
                    }
                }
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ((pmtADCValue[i] * ModuleGain[i]) / max);
                }
            }

            for (int i = 0; i < XposArray.Length; i++)
            {

                for (int j = 0; j < PmtCount; j++)
                {
                    val += LogMue[i][j] * normalizePMTValue[j];
                }

                val -= SumMu[i];
                if (val > maxChk)
                {
                    maxChk = val;
                    pos[0] = XposArray[i];
                    pos[1] = YposArray[i];
                    pos[2] = ZposArray[i];
                }
                val = 0;
            }

            point.X = pos[0] + ModuleOffsetX;
            point.Y = pos[1] + ModuleOffsetY;
            point.Z = ModuleOffsetZ;


            //sw.Stop();
            //Debug.WriteLine("0: vvvvvvvvvvvvvvvvvvvvvvvvvvvv " + sw.ElapsedTicks);
            return (point, eCalEnergy);
        }

        private double[][][] XYLogMue;
        private double[][] XYSumMu;
        private int sizeX;
        private int sizeY;
        private void MuArrays()
        {
            int minX = (int)(from lut in LUT
                             select lut).Min(lut => lut.Xpos);
            int maxX = (int)(from lut in LUT
                             select lut).Max(lut => lut.Xpos);
            int minY = (int)(from lut in LUT
                             select lut).Min(lut => lut.Ypos);
            int maxY = (int)(from lut in LUT
                             select lut).Max(lut => lut.Ypos);

            sizeX = maxX - minX + 1;
            sizeY = maxY - minY + 1;

            XYLogMue = new double[sizeX][][];
            XYSumMu = new double[sizeX][];

            Stopwatch sw = new Stopwatch();
            sw.Start();
            var yascending = (from lut in LUT
                              orderby lut.Ypos ascending
                              select lut).ToArray();
            var xyascending = (from lut in yascending
                              orderby lut.Xpos ascending
                              select lut).ToArray();

            int i = 0;
             for(int x = 0; x<sizeX;x++)
             {
                 XYLogMue[x] = new double[sizeY][];
                 XYSumMu[x] = new double[sizeY];
                 for (int y = 0; y < sizeY; y++)
                 {
                    try
                    {
                        XYLogMue[x][y] = xyascending[i].LogMu.ToArray();
                        XYSumMu[x][y] = xyascending[i].SumMu;
                    }
                    catch{ }
                    finally { i++; }
                 }
             }

            //Parallel.For(0, sizeX, x =>
            // {
            //     XYLogMue[x] = new double[sizeY][];
            //     XYSumMu[x] = new double[sizeY];
            //     for (int y = 0; y < sizeY; y++)
            //     {
            //         var selLUT = (from lut in LUT
            //                      where lut.Xpos == minX + x && lut.Ypos == minY + y
            //                      select lut).ToArray();
            //         if (selLUT.Any())
            //         {
            //             XYLogMue[x][y] = selLUT.First().LogMu.ToArray();
            //             XYSumMu[x][y] = selLUT.First().SumMu;
            //         }

            //     }
            // });
            sw.Stop();
            Trace.WriteLine("XY Logmu Table is done. Takes: " + sw.ElapsedMilliseconds + " ms");
        }

        /// <summary>
        /// Fast MLPE
        /// </summary>
        /// <param name="pmtADCValue">PMT Value in short array </param>
        /// <returns> {xpos,ypos,zpos,Energy}[mm,mm,mm,keV]</returns>
        public (Point3D, double) FastPosAndEEstimate(short[] pmtADCValue)
        {
            //Stopwatch sw = new Stopwatch();
            //sw.Start();
            Point3D point = new Point3D();
            double eCalEnergy = GetEcal(pmtADCValue);

            double max = 0;
            double valMaxChk = -50000;
            double val = 0;
            double[] pos = new double[3];
            double[] normalizePMTValue = new double[PmtCount];


            //sw.Stop();
            //Debug.WriteLine("0: " + sw.ElapsedTicks);
            //sw.Restart();
            if (ModulePMTOrder.IsOrderChange)
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    if (pmtADCValue[ModulePMTOrder.Order[i]] * ModuleGain[i] > max)
                    {
                        max = pmtADCValue[ModulePMTOrder.Order[i]] * ModuleGain[i];
                    }
                }
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ((pmtADCValue[ModulePMTOrder.Order[i]] * ModuleGain[i]) / max);
                }
            }
            else
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    if (pmtADCValue[i] > max)
                    {
                        max = pmtADCValue[i]* ModuleGain[i];
                    }
                }
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ( (pmtADCValue[i] * ModuleGain[i]) / max);
                }
            }

            int[] Max1 = new int[2];
            int GridSize1 = sizeX / 5;
            for (int x1 = GridSize1; x1 < sizeX; x1 += GridSize1)
            {
                for (int y1 = GridSize1; y1 < sizeY; y1 += GridSize1)
                {
                    if (double.IsNaN(XYLogMue[x1][y1][0]))
                        continue;
                    for (int j = 0; j < PmtCount; j++)
                    {
                        if (XYLogMue[x1][y1] != null)
                            val += XYLogMue[x1][y1][j] * normalizePMTValue[j];
                    }
                    val -= XYSumMu[x1][y1];
                    if (val > valMaxChk)
                    {
                        valMaxChk = val;
                        Max1[0] = x1;
                        Max1[1] = y1;
                    }
                    val = 0;
                }
            }
            valMaxChk = -5000;

            int[] Max2 = new int[2];
            int GridSize2 = sizeX / 20;
            for (int x2 = Max1[0] - GridSize1; x2 < Math.Min(Max1[0] + GridSize1,sizeX); x2 += GridSize2)
            {
                if (x2 > -1)
                {

                    for (int y2 = Max1[1] - GridSize1; y2 < Math.Min( Max1[1] + GridSize1,sizeY); y2 += GridSize2)
                    {
                        if (y2 > -1)
                        {
                            if (double.IsNaN(XYLogMue[x2][y2][0]))
                                continue;
                            for (int j = 0; j < PmtCount; j++)
                            {
                                if (XYLogMue[x2][y2] != null)
                                    val += XYLogMue[x2][y2][j] * normalizePMTValue[j];
                            }
                            
                            val -= XYSumMu[x2][y2];
                            if (val > valMaxChk)
                            {
                                valMaxChk = val;
                                Max2[0] = x2;
                                Max2[1] = y2;
                            }
                            val = 0;

                        }
                    }

                }
            }
            valMaxChk = -5000;


            int[] Max3 = new int[2];
            int GridSize3 = 1;
            for (int x2 = Max2[0] - GridSize2; x2 <Math.Min( Max2[0] + GridSize2,sizeX); x2 += GridSize3)
            {
                if (x2 > -1)
                {
                    for (int y2 = Max2[1] - GridSize2; y2 <Math.Min( Max2[1] + GridSize2,sizeY); y2 += GridSize3)
                    {
                        if (y2 > -1)
                        {
                            if (double.IsNaN(XYLogMue[x2][y2][0]))
                                continue;
                            for (int j = 0; j < PmtCount; j++)
                            {
                                if (XYLogMue[x2][y2] == null) continue;
                                val += XYLogMue[x2][y2][j] * normalizePMTValue[j];
                            }
                            
                            val -= XYSumMu[x2][y2];
                            if (val > valMaxChk)
                            {
                                valMaxChk = val;
                                Max3[0] = x2;
                                Max3[1] = y2;
                            }
                            val = 0;
                        }
                    }
                }
            }

            point.X = Convert.ToDouble(Max3[1] - sizeY / 2) / 1000 + ModuleOffsetX; //mm to meter
            point.Y = Convert.ToDouble(Max3[0] - sizeX / 2) / 1000 + ModuleOffsetY;
            point.Z = ModuleOffsetZ;


            //sw.Stop();
            //Debug.WriteLine("0: vvvvvvvvvvvvvvvvvvvvvvvvvvvv " + sw.ElapsedTicks);
            return (point, eCalEnergy);
        }

        public double GetEcal(short[] pmtADCValue)
        {
            double eCalEnergy = 0;

            double[] arrangedPMTValue = new double[PmtCount];

            if (ModulePMTOrder.IsOrderChange)
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    arrangedPMTValue[i] = (pmtADCValue[ModulePMTOrder.Order[i]]);
                }
            }

            for (int i = 0; i < PmtCount; i++)
            {
                eCalEnergy += arrangedPMTValue[i] * ModuleGain[i];
            }
            eCalEnergy += ModuleGain[^1];
            eCalEnergy = ModuleEcalData.a * eCalEnergy * eCalEnergy + ModuleEcalData.b * eCalEnergy + ModuleEcalData.c;

            return eCalEnergy;
        }
    }
}
