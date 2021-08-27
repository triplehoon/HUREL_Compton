using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using HUREL.Compton;

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

    public class LACC_Control
    {        
        /// <summary>
        /// Module Information
        /// </summary>
        public ModuleInfo Module;
        /// <summary>
        /// FPGA Channel 0 to 7. For mono index 0 is Channel 0 to 3
        /// </summary>
        public LACC_Module[/*Channel Numer*/] LACC_Scatter_Modules;
        /// <summary>
        /// FPGA Channel 8 to 15. For mono index 0 is Channel 8 to 11
        /// </summary>
        public LACC_Module[/*Channel Numer*/] LACC_Absorber_Modules; 
        /// <summary>
        /// List of list-mode data
        /// </summary>
        public List<LMData> ListedLMData = new List<LMData>();


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

        public static int[] debugCountScatter = new int[4] {0,0,0,0 };
        public static int[] debugCountAbsorber = new int[4] { 0, 0, 0, 0 };
        public void AddListModeData(ushort[] fullADCArrayValue, Matrix3D deviceTransformation, List<AddListModeDataEchk> Echks, bool isMLPEOn = false)
        {

            LMData lmData;
            if (Module == ModuleInfo.Mono)
            {
                ushort[] scatter = fullADCArrayValue[0..36];
                ushort[] absorber = fullADCArrayValue[72..108];

                int scatterProd = 1;
                foreach(ushort s in scatter)
                {
                    scatterProd *= s;
                    if (scatterProd == 0)
                        break;
                    scatterProd = 1;
                }

                int absorberProd = 1;
                foreach (ushort s in absorber)
                {
                    absorberProd *= s;
                    if (absorberProd == 0)
                        break;
                    absorberProd = 1;
                }

                if(scatterProd != 0 && absorberProd != 0)
                {
                    double scatterEcal = LACC_Scatter_Modules[0].GetEcal(scatter);
                    LACC_Scatter_Modules[0].SpectrumEnergy.AddEnergy(scatterEcal);
                    double absorberEcal = LACC_Absorber_Modules[0].GetEcal(absorber);
                    LACC_Absorber_Modules[0].SpectrumEnergy.AddEnergy(absorberEcal);

                    double combinedEcal = scatterEcal + absorberEcal;
                    foreach (var E in Echks)
                    {
                        if (combinedEcal > E.MinE && combinedEcal < E.MaxE && isMLPEOn)
                        {
                            var scatterMLPEdata = LACC_Scatter_Modules[0].FastPosEstimate(scatter);
                            var absorberMLPEdata = LACC_Absorber_Modules[0].FastPosEstimate(absorber);
                            lmData = new LMData(scatterMLPEdata, absorberMLPEdata, scatterEcal, absorberEcal, deviceTransformation);
                            ListedLMData.Add(lmData);
                        }
                    }
                }
                
                else if(scatterProd != 0 && absorberProd ==0)
                {
                    double scatterEcal = LACC_Scatter_Modules[0].GetEcal(scatter);
                    LACC_Scatter_Modules[0].SpectrumEnergy.AddEnergy(scatterEcal);

                    foreach (var E in Echks)
                    {
                        if (scatterEcal > E.MinE && scatterEcal < E.MaxE && isMLPEOn)
                        {
                            var scatterMLPEdata = LACC_Scatter_Modules[0].FastPosEstimate(scatter);
                            lmData = new LMData(scatterMLPEdata, scatterEcal, deviceTransformation);
                            ListedLMData.Add(lmData);
                        }
                    }
                    
                }
                
                else if(scatterProd == 0 && absorberProd != 0)
                {
                    double absorberEcal = LACC_Absorber_Modules[0].GetEcal(absorber);
                    LACC_Absorber_Modules[0].SpectrumEnergy.AddEnergy(absorberEcal);
                }
                else
                {
                    //Do nothing
                }
            }
            else if (Module == ModuleInfo.QuadSingleHead)
            {
                ushort[][] scatterShorts = new ushort[4][];
                scatterShorts[0] = fullADCArrayValue[0..9];
                scatterShorts[1] = fullADCArrayValue[9..18];
                scatterShorts[2] = fullADCArrayValue[18..27];
                scatterShorts[3] = fullADCArrayValue[27..36];
                double[] scattersEnergy = new double[4];

                ushort[][] absorberShorts = new ushort[4][];
                absorberShorts[0] = fullADCArrayValue[72..81];
                absorberShorts[1] = fullADCArrayValue[81..90];
                absorberShorts[2] = fullADCArrayValue[90..99];
                absorberShorts[3] = fullADCArrayValue[99..108];
                double[] absorbersEnergy = new double[4];
                int scatterInteractionCount = 0;
                int absorberInteractionCount = 0;
                int scatterInteractModuleNum = 4;
                int absorberInteractModuleNum = 4;
                for (int i = 0; i < 4; ++i)
                {
                    scattersEnergy[i] = LACC_Scatter_Modules[i].GetEcal(scatterShorts[i]);
                    if (scattersEnergy[i] > 0)
                    {
                        LACC_Scatter_Modules[i].SpectrumEnergy.AddEnergy(scattersEnergy[i]);
                        scatterInteractModuleNum = i;
                        ++scatterInteractionCount;
                        debugCountScatter[i]++;
                    }
                    absorbersEnergy[i] = LACC_Absorber_Modules[i].GetEcal(absorberShorts[i]);
                    if (absorbersEnergy[i] > 0)
                    {
                        LACC_Absorber_Modules[i].SpectrumEnergy.AddEnergy(absorbersEnergy[i]);
                        absorberInteractModuleNum = i;
                        ++absorberInteractionCount;
                        debugCountAbsorber[i]++;
                    }
                }

                if (scatterInteractionCount == 1)
                {
                    if (absorberInteractionCount == 1)
                    {
                        //Compton
                        foreach (var e in Echks)
                        {
                            if (e.MinE < absorbersEnergy[absorberInteractModuleNum] + scattersEnergy[scatterInteractModuleNum] && e.MaxE > scattersEnergy[scatterInteractModuleNum] + absorbersEnergy[absorberInteractModuleNum])
                            {
                                ListedLMData.Add(new LMData(LACC_Scatter_Modules[scatterInteractModuleNum].FastPosEstimate(scatterShorts[scatterInteractModuleNum]), LACC_Absorber_Modules[absorberInteractModuleNum].FastPosEstimate(absorberShorts[absorberInteractModuleNum]),
                                                        scattersEnergy[scatterInteractModuleNum], absorbersEnergy[absorberInteractModuleNum], deviceTransformation));
                            }
                        }                  
                    }
                    else if (absorberInteractionCount == 0)
                    {
                        //Coded Apature
                        foreach (var e in Echks)
                        {
                            if (e.MinE < scattersEnergy[scatterInteractModuleNum] && e.MaxE > scattersEnergy[scatterInteractModuleNum])
                            {
                                ListedLMData.Add(new LMData(LACC_Scatter_Modules[scatterInteractModuleNum].FastPosEstimate(scatterShorts[scatterInteractModuleNum]),
                                scattersEnergy[scatterInteractModuleNum], deviceTransformation));
                            }
                        }
                     
                    }
                    else
                    {
                        return;
                    }
                }
                //else if (absorberInteractionCount == 1)
                //{
                //    ////absorber test code
                //    //foreach (var e in Echks)
                //    //{
                //    //    if (e.MinE < absorbersEnergy[absorberInteractModuleNum] && e.MaxE > absorbersEnergy[absorberInteractModuleNum])
                //    //    {
                //    //        ListedLMData.Add(new LMData(new Point3D(0,0,0), LACC_Absorber_Modules[absorberInteractModuleNum].FastPosEstimate(absorberShorts[absorberInteractModuleNum]),
                //    //                                10, absorbersEnergy[absorberInteractModuleNum], deviceTransformation));
                //    //    }
                //    //}
                //}
                else
                {
                    return;
                }        
            }
            else //(Module == ModuleInfo.QuadDualHead)
            {
               throw new NotImplementedException();
            }
        }

        public void SaveListmodeData(string path, string fileName)
        {
            string csvPath = Path.Combine(path.ToString(), DateTime.Now.ToString("yyyyMMddHHmm") + "_"+ fileName + "_lmData.csv");
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(csvPath))
            {
                //file.WriteLine("Time[HHMMssFFF],SCposX[m],SCposY,SCposZ,SCEnergy[keV],ABposX,ABposY,ABposZ,ABEnergy");

                foreach (var lmdata in ListedLMData)
                {
                    if (lmdata.AbsorberLMDataInfo != null && lmdata.ScatterLMDataInfo != null)
                    {
                        file.WriteLine($"{lmdata.MeasurementTime.ToString("HHMMssfff")},{lmdata.ScatterLMDataInfo.RelativeInteractionPoint3D.X},{lmdata.ScatterLMDataInfo.RelativeInteractionPoint3D.Y},{lmdata.ScatterLMDataInfo.RelativeInteractionPoint3D.Z},{lmdata.ScatterLMDataInfo.InteractionEnergy}," +
                        $",{lmdata.AbsorberLMDataInfo.RelativeInteractionPoint3D.X},{lmdata.AbsorberLMDataInfo.RelativeInteractionPoint3D.Y},{lmdata.AbsorberLMDataInfo.RelativeInteractionPoint3D.Z},{lmdata.AbsorberLMDataInfo.InteractionEnergy}");
                    }
                    else if (lmdata.AbsorberLMDataInfo != null && lmdata.ScatterLMDataInfo == null)
                    {
                        file.WriteLine($"{lmdata.MeasurementTime.ToString("HHMMssFFF")},NaN,NaN,NaN,NaN," +
                        $",{lmdata.AbsorberLMDataInfo.RelativeInteractionPoint3D.X},{lmdata.AbsorberLMDataInfo.RelativeInteractionPoint3D.Y},{lmdata.AbsorberLMDataInfo.RelativeInteractionPoint3D.Z},{lmdata.AbsorberLMDataInfo.InteractionEnergy}");
                    }
                    else if (lmdata.AbsorberLMDataInfo == null && lmdata.ScatterLMDataInfo != null)
                    {
                        file.WriteLine($"{lmdata.MeasurementTime.ToString("HHMMssFFF")},{lmdata.ScatterLMDataInfo.RelativeInteractionPoint3D.X},{lmdata.ScatterLMDataInfo.RelativeInteractionPoint3D.Y},{lmdata.ScatterLMDataInfo.RelativeInteractionPoint3D.Z},{lmdata.ScatterLMDataInfo.InteractionEnergy}," +
                      ",NaN,NaN,NaN,NaN");
                    }
                }
            }
        }

        public void ResetLMData()
        {
            ListedLMData = new List<LMData>();
            foreach(var m in LACC_Scatter_Modules)
            {
                m.SpectrumEnergy.Reset();
            }
            foreach (var m in LACC_Absorber_Modules)
            {
                m.SpectrumEnergy.Reset();
            }
            LMData.Reset();
        }

        public void ResetModuleEnergy()
        {
            foreach (var m in LACC_Scatter_Modules)
            {
                m.SpectrumEnergy.Reset();
            }
            foreach (var m in LACC_Absorber_Modules)
            {
                m.SpectrumEnergy.Reset();
            }
        }
    }

    public record AddListModeDataEchk(double MinE, double MaxE);


    public class LACC_Module
    {
        public int ChannelNumber;

        public SpectrumEnergy SpectrumEnergy { get; init; }

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


        public double[] EnergyGain { get; set; }
        public double[] MLPEGain { get; set; }

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
            public double Xpos { get; set; }
            public double Ypos { get; set; }
            public double Zpos { get; set; }

            public List<double> LogMu { get; set; }
            public double SumMu { get; set; }
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
        public LACC_Module(ModuleInfo mode, ModuleOffset offset, EcalVar ecalData, double[] eGain, double[] mlpeGain, ModulePMTOrderInfo pmtOrder, string csvFileLUT, int channelNumber = 0, int spectrumBinSize = 5, double spectrumMaxE = 2000)
        {
            SetupModuleInfo = mode;
            ChannelNumber = channelNumber;
            ModuleOffetData = offset;
            ModuleOffsetX = ModuleOffetData.x;
            ModuleOffsetY = ModuleOffetData.y;
            ModuleOffsetZ = ModuleOffetData.z;
            ModuleEcalData = ecalData;

            EnergyGain = eGain;
            MLPEGain = mlpeGain;
            ModulePMTOrder = pmtOrder;

            loadLUT(csvFileLUT);
            MuArrays();

            SpectrumEnergy = new SpectrumEnergy(spectrumBinSize, spectrumMaxE);
        }

        static public double[] LoadGain(string fileName)
        {
            if (!File.Exists(fileName))
            {
                throw new ArgumentException();
            }
            using (StreamReader IO = new StreamReader(fileName))
            {
                var line = IO.ReadLine();
                double[] value = Array.ConvertAll(line.Split(','), Double.Parse);
                return value;                
            }
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
                if (SetupModuleInfo == ModuleInfo.Mono)
                {
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
        public (Point3D, double) PosAndEEstimate(ushort[] pmtADCValue)
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
                    if (pmtADCValue[ModulePMTOrder.Order[i]] * MLPEGain[i] > max)
                    {
                        max = pmtADCValue[ModulePMTOrder.Order[i]] * MLPEGain[i];
                    }
                }
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ((pmtADCValue[ModulePMTOrder.Order[i]] * MLPEGain[i]) / max);
                }
            }
            else
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    if (pmtADCValue[i] * MLPEGain[i] > max)
                    {
                        max = pmtADCValue[i] * MLPEGain[i];
                    }
                }
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ((pmtADCValue[i] * MLPEGain[i]) / max);
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
            if (SetupModuleInfo == ModuleInfo.QuadSingleHead)
            {
                for (int k = 0; k < LUT.Count; ++k)
                {
                    MLPELUTElement templut = LUT[k];
                    LUT[k] = new MLPELUTElement() { Xpos = templut.Xpos, Ypos = -templut.Ypos, Zpos = templut.Zpos, LogMu = templut.LogMu, SumMu = templut.SumMu };
                }
            }

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
             for(int x = 0; x < sizeX; x++)
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
        /// Fast MLPE return x, y, z position and energy
        /// </summary>
        /// <param name="pmtADCValue">PMT Value in ushort array </param>
        /// <returns> {xpos,ypos,zpos,Energy}[mm,mm,mm,keV]</returns>
        public Point3D FastPosEstimate(ushort[] pmtADCValue)
        {
            //Stopwatch sw = new Stopwatch();
            //sw.Start();
            Point3D point = new Point3D();

            double valMaxChk = -50000;
            double[] normalizePMTValue = new double[PmtCount];


            //sw.Stop();
            //Debug.WriteLine("0: " + sw.ElapsedTicks);
            //sw.Restart();
            if (ModulePMTOrder.IsOrderChange)
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ((pmtADCValue[ModulePMTOrder.Order[i]] * MLPEGain[i]));
                }
            }
            else
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    normalizePMTValue[i] = ( (pmtADCValue[i] * MLPEGain[i]));
                }
            }

            int[] Max1 = new int[2];
            int GridSize1 = sizeX / 10;
            for (int x1 = GridSize1; x1 < sizeX; x1 += GridSize1)
            {
                for (int y1 = GridSize1; y1 < sizeY; y1 += GridSize1)
                {
                    double val = 0;

                    if (double.IsNaN(XYLogMue[x1][y1][0]))
                        continue;
                    for (int j = 0; j < PmtCount; j++)
                    {
                        if (XYLogMue[x1][y1] == null)
                        {
                            continue;
                        }

                        val += XYLogMue[x1][y1][j] * normalizePMTValue[j];

                    }
                    //val -= XYSumMu[x1][y1];
                    if (val > valMaxChk)
                    {
                        valMaxChk = val;
                        Max1[0] = x1;
                        Max1[1] = y1;
                    }
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
                            double val = 0;

                            if (double.IsNaN(XYLogMue[x2][y2][0]))
                                continue;
                            for (int j = 0; j < PmtCount; j++)
                            {
                                if (XYLogMue[x2][y2] == null)
                                {
                                    continue;
                                }
                                 val += XYLogMue[x2][y2][j] * normalizePMTValue[j];
                            }
                            
                            //val -= XYSumMu[x2][y2];
                            if (val > valMaxChk)
                            {
                                valMaxChk = val;
                                Max2[0] = x2;
                                Max2[1] = y2;
                            }

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
                            double val = 0;

                            if (double.IsNaN(XYLogMue[x2][y2][0]))
                                continue;
                            for (int j = 0; j < PmtCount; j++)
                            {
                                if (XYLogMue[x2][y2] == null)
                                {
                                    continue;
                                }                                    
                                val += XYLogMue[x2][y2][j] * normalizePMTValue[j];
                            }
                            
                           // val -= XYSumMu[x2][y2];
                            if (val > valMaxChk)
                            {
                                valMaxChk = val;
                                Max3[0] = x2;
                                Max3[1] = y2;
                            }
                        }
                    }
                }
            }

            point.X = (Convert.ToDouble(Max3[1]) - sizeY / 2) / 1000 + ModuleOffsetX; //mm to meter
            point.Y = (Convert.ToDouble(Max3[0]) - sizeX / 2) / 1000 + ModuleOffsetY;
            point.Z = ModuleOffsetZ;


            //sw.Stop();
            //Debug.WriteLine("0: vvvvvvvvvvvvvvvvvvvvvvvvvvvv " + sw.ElapsedTicks);
            return point;
        }

        public double GetEcal(ushort[] pmtADCValue)
        {
            double eCalEnergy = 0;
            ushort checkZero = 0;

            double[] arrangedPMTValue = new double[PmtCount];
            

            if (ModulePMTOrder.IsOrderChange)
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    checkZero += pmtADCValue[i]; 
                    arrangedPMTValue[i] = (pmtADCValue[ModulePMTOrder.Order[i]]);
                }

                for (int i = 0; i < PmtCount; i++)
                {
                    eCalEnergy += arrangedPMTValue[i] * EnergyGain[i];
                }
                eCalEnergy += EnergyGain[^1];
            }
            else
            {
                for (int i = 0; i < PmtCount; i++)
                {
                    eCalEnergy += pmtADCValue[i] * EnergyGain[i];
                    checkZero += pmtADCValue[i];

                }
                eCalEnergy += EnergyGain[^1];
            }
            if (checkZero == 0)
            {
                return 0;
            }


            eCalEnergy = ModuleEcalData.a * eCalEnergy * eCalEnergy + ModuleEcalData.b * eCalEnergy + ModuleEcalData.c;            
            return eCalEnergy;
        }
    }
}
