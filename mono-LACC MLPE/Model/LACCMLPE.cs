using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL.Compton
{
    class LACCMLPE
    {
        private int ScintilatorNumber;
        private MLPEMode SetupMLPEMode;
        private int pmtCount;
        public enum MLPEMode
        {
            Quad  =9,
            Mono = 36
        }
        /// <summary>
        /// ax^2+bx+c
        /// </summary>
        public struct EcalVar
        {
            public double a;
            public double b;
            public double c;
        }

        /// <summary>
        /// x, y, z offset in [mm]
        /// </summary>
        public struct ChOffet
        {
            public double x;
            public double y;
            public double z;
        }



        public ChOffet ChOffetData;
        private double ChOffsetX;
        private double ChOffsetY;
        private double ChOffsetZ;

        public EcalVar ChEcalData;
        public double[] ChGain;
        private double EcalVarA;
        private double EcalVarB;
        private double EcalVarC;

        private List<MLPELUTElement> LUT = new List<MLPELUTElement>();
        private int LUTSize =0;
        
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
        private double[]    XposArray;
        private double[]    YposArray;
        private double[]    ZposArray;
        private double[][]   LogMue;
        private double[]    SumMu;



        /// </summary>
        /// <param name="mode"> Quad, Mono</param>
        /// <param name="offset"> offset. [mm]</param>
        /// <param name="ecalData">ax^2+bx+c</param>
        /// <param name="gain"> Scintilator check. </param>    
        /// <param name="scintilatorNumber"> Scintilator check. </param>    
        /// <param name="csvFileLUT"> cvsFile link </param>    
        public LACCMLPE(MLPEMode mode, ChOffet offset, EcalVar ecalData, double[] gain, string csvFileLUT, int scintilatorNumber = 0)
        {                        
            SetupMLPEMode = mode;
            ScintilatorNumber = scintilatorNumber;
            ChOffetData = offset;
            ChOffsetX = ChOffetData.x;
            ChOffsetY = ChOffetData.y;
            ChOffsetZ = ChOffetData.z;
            ChEcalData = ecalData;
            EcalVarA = ChEcalData.a;
            EcalVarB = ChEcalData.b;
            EcalVarC = ChEcalData.c;
            ChGain = gain;           
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
            pmtCount = (int)SetupMLPEMode;
            try
            {
                while (!IO.EndOfStream)
                {
                    var line = IO.ReadLine();
                    double[] value = Array.ConvertAll(line.Split(','),Double.Parse);
                    var test = value[3..(int)(pmtCount + 3)];
                    MLPELUTElement element = new MLPELUTElement { Xpos = value[0], Ypos = value[1], Zpos = value[2], LogMu = new List<double>(value[3..(pmtCount+3)]), SumMu = value[value.Length -1] };
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
                    XposArray[i] = lut.Xpos;
                    YposArray[i] = lut.Ypos;
                    ZposArray[i] = lut.Zpos;
                    LogMue[i] = new double[pmtCount];
                    for (int j = 0; j < pmtCount; j++)
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
        }      
        
        /// <summary>
        /// 
        /// </summary>
        /// <param name="pmtADCValue">PMT Value in short array </param>
        /// <returns> {xpos,ypos,zpos,Energy}[mm,mm,mm,keV]</returns>
        public double[] PosAndEEstimate(short[] pmtADCValue)
        {
            //Stopwatch sw = new Stopwatch();
            //sw.Start();

            double[] positionEnergyArray = new double[4];
            double eCalEnergy = 0;
            double max = 0;            
            double maxChk = -50000;
            double val = 0;
            double[] pos = new double[3];
            double[] normalizePMTValue = new double[pmtCount];


            for (int i = 0; i < pmtCount; i++)
            {
                eCalEnergy +=pmtADCValue[i] * ChGain[i];
            }
            eCalEnergy += eCalEnergy + ChGain[ChGain.Length - 1];
            eCalEnergy = EcalVarA * eCalEnergy * eCalEnergy + EcalVarB * eCalEnergy + EcalVarC;
            //sw.Stop();
            //Debug.WriteLine("0: " + sw.ElapsedTicks);
            //sw.Restart();

            for (int i = 0; i < pmtCount; i++)
            {
                if(pmtADCValue[i] > max)
                {
                    max =pmtADCValue[i];
                }
            }

            for (int i = 0; i < pmtCount; i++)
            {
                normalizePMTValue[i] = (pmtADCValue[i] / max);
            }


            for (int i = 0; i < XposArray.Length; i++)
            {

                for (int j = 0; j < pmtCount; j++)
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

            positionEnergyArray[0] = pos[0] + ChOffsetX;
            positionEnergyArray[1] = pos[1] + ChOffsetY;
            positionEnergyArray[2] = ChOffsetZ;
            positionEnergyArray[3] = eCalEnergy;

            //sw.Stop();
            //Debug.WriteLine("0: vvvvvvvvvvvvvvvvvvvvvvvvvvvv " + sw.ElapsedTicks);
            return positionEnergyArray;
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
            for (int x = 0; x < sizeX; x++)
            {
                XYLogMue[x] = new double[sizeY][];
                XYSumMu[x] = new double[sizeY];
                Parallel.For(0,sizeY,y=>
                {
                    var selLUT = from lut in LUT
                                 where lut.Xpos == minX+x && lut.Ypos == minY+y
                                 select lut;
                    if (selLUT.Any()) 
                    {
                        XYLogMue[x][y] = selLUT.First().LogMu.ToArray();
                        XYSumMu[x][y] = selLUT.First().SumMu;
                    }

                });
            }
            sw.Stop();
            Trace.WriteLine(sw.ElapsedMilliseconds);
            Debug.WriteLine("XY Logmu Table is done.");
        }

        /// <summary>
        /// Fast MLPE
        /// </summary>
        /// <param name="pmtADCValue">PMT Value in short array </param>
        /// <returns> {xpos,ypos,zpos,Energy}[mm,mm,mm,keV]</returns>
        public double[] FastPosAndEEstimate(short[] pmtADCValue)
        {
            //Stopwatch sw = new Stopwatch();
            //sw.Start();

            double[] positionEnergyArray = new double[4];
            double eCalEnergy = 0;
            double max = 0;
            double valMaxChk = -50000;
            double val = 0;
            double[] pos = new double[3];
            double[] normalizePMTValue = new double[pmtCount];

            for (int i = 0; i < pmtCount; i++)
            {
                eCalEnergy += pmtADCValue[i] * ChGain[i];
            }
            eCalEnergy += eCalEnergy + ChGain[ChGain.Length - 1];
            eCalEnergy = EcalVarA * eCalEnergy * eCalEnergy + EcalVarB * eCalEnergy + EcalVarC;
            //sw.Stop();
            //Debug.WriteLine("0: " + sw.ElapsedTicks);
            //sw.Restart();

            for (int i = 0; i < pmtCount; i++)
            {
                if (pmtADCValue[i] > max)
                {
                    max = pmtADCValue[i];
                }
            }

            for (int i = 0; i < pmtCount; i++)
            {
                normalizePMTValue[i] = (pmtADCValue[i] / max);
            }

            int[] Max1 = new int[2];
            int GridSize1 = sizeX / 5;
            for (int x1 = GridSize1; x1 < sizeX; x1 += GridSize1)
            {
                for (int y1 = GridSize1; y1 < sizeY; y1 += GridSize1)
                {
                    for (int j = 0; j < pmtCount; j++)
                    {
                        if (XYLogMue[x1][y1] != null)
                        val += XYLogMue[x1][y1][j] * normalizePMTValue[j];
                    }
                    if (XYSumMu[x1][y1] != 0)
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
            int GridSize2 = sizeX / 25;
            for (int x2 = Max1[0] - GridSize1; x2 < Max1[0] + GridSize1; x2 += GridSize2)
            {
                if (x2 > -1)
                {

                    for (int y2 = Max1[1] - GridSize1; y2 < Max1[1] + GridSize1; y2 += GridSize2)
                    {
                        if (y2 > -1)
                        {
                            for (int j = 0; j < pmtCount; j++)
                            {
                                if (XYLogMue[x2][y2] != null)
                                    val += XYLogMue[x2][y2][j] * normalizePMTValue[j];
                            }
                            if (XYSumMu[x2][y2] != 0)
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
            for (int x2 = Max2[0] - GridSize2; x2 < Max2[0] + GridSize2; x2 += GridSize3)
            {
                if (x2 > -1)
                {
                    for (int y2 = Max2[1] - GridSize2; y2 < Max2[1] + GridSize2; y2 += GridSize3)
                    {
                        if (y2 > -1)
                        {
                            for (int j = 0; j < pmtCount; j++)
                            {
                                if (XYLogMue[x2][y2] == null) break;
                                val += XYLogMue[x2][y2][j] * normalizePMTValue[j];
                            }
                            if (XYSumMu[x2][y2] == 0) break;
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

            positionEnergyArray[0] = Max3[0] - sizeX / 2 + ChOffsetX;
            positionEnergyArray[1] = Max3[1] - sizeY / 2 + ChOffsetY;
            positionEnergyArray[2] = ChOffsetZ;
            positionEnergyArray[3] = eCalEnergy;

            //sw.Stop();
            //Debug.WriteLine("0: vvvvvvvvvvvvvvvvvvvvvvvvvvvv " + sw.ElapsedTicks);
            return positionEnergyArray;
        }
    }
}
