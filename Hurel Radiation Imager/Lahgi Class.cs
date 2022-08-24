﻿using System.IO;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;


namespace HUREL.Compton
{
    public enum eLahgiApiEnvetArgsState
    {
        Loading,
        Slam
    }
    public class LahgiApiEnvetArgs:EventArgs
    {
        public eLahgiApiEnvetArgsState State { get; private set; }
        public LahgiApiEnvetArgs(eLahgiApiEnvetArgsState state)
        {
            State = state;
        }
    }
    public static class LahgiApi
    {
        private static ILog log = LogManager.GetLogger(typeof(LahgiApi));

        private static CRUXELLLACC fpga;
        private static LahgiWrapper lahgiWrapper;
        private static RtabmapWrapper rtabmapWrapper;
        public static CRUXELLLACC.VariableInfo fpgaVariables;

        public static EventHandler? StatusUpdate;
        public static void StatusUpdateInvoke(object? obj, EventArgs args)
        {
            StatusUpdate?.Invoke(null, new LahgiApiEnvetArgs(eLahgiApiEnvetArgsState.Slam));

        }
        private static string statusMsg = "";
        public static string StatusMsg
        {
            get
            {
                return statusMsg;
            }
       
            private set
            {
                log = LogManager.GetLogger("LahgiApi");
                log.Info(value);
                statusMsg = value;
                StatusUpdateInvoke(null, EventArgs.Empty);
            }
        }
        public static bool IsSavingBinary
        {
            get { return fpga.IsSavingBinaryData; }
            set { fpga.IsSavingBinaryData = value; }
        }
        public static bool IsLahgiInitiate { get; private set; }
        public static bool IsRtabmapInitiate { get; private set; }
        
        public static bool IsInitiate
        {
            get
            {
                return IsLahgiInitiate && IsRtabmapInitiate;
            }
        }

        /// <summary>
        /// Start Stop Counting, Get Spectrum, Get 3D image, Get 2D image, 
        /// Start and stop imaging
        /// </summary>
        static LahgiApi()
        {
            fpga = new CRUXELLLACC();
            fpgaVariables = fpga.Variables;
            fpga.USBChangeHandler += UpdateDeviceList;
            IsFpgaAvailable = false;
            lahgiWrapper = new LahgiWrapper();
            rtabmapWrapper = new RtabmapWrapper();
            StatusMsg = "Wrappers loaded";

            StatusUpdateInvoke(null, EventArgs.Empty);
            IsSavingBinary = false;
            UpdateDeviceList(null, EventArgs.Empty);
        }                
        public static bool InitiateLaghi()
        {
            StatusMsg = "Initiating LAHGI";
            
            if (lahgiWrapper.Initiate(eModuleManagedType.QUAD))
            {
                StatusMsg = "Successfully initiate Lahgi";
                IsLahgiInitiate = true;
                StatusUpdateInvoke(null, EventArgs.Empty);
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Lahgi";
                IsLahgiInitiate = false;
                StatusUpdateInvoke(null, EventArgs.Empty);
                return false;
            }
        }
        public static bool InititateRtabmap()
        {

            StatusMsg = "Initiating RTABAMP";

            if (rtabmapWrapper.InitiateRtabmap())
            {
                StatusMsg = "Successfully initiate Rtabmap";
                IsRtabmapInitiate = true;
                StatusUpdateInvoke(null, EventArgs.Empty);
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Rtabmap";
                IsRtabmapInitiate = false;
                StatusUpdateInvoke(null, EventArgs.Empty);
                return false;
            }
        }
       
        public static BitmapImage? GetRgbImage()
        {
            BitmapImage? img = null; 
            if (!IsRtabmapInitiate)
            {
                return img;
            }

            int width = 1;
            int height = 1;
            int stride = 1;

            IntPtr data = IntPtr.Zero;
            rtabmapWrapper.GetRealTimeRGB(ref width, ref height, ref stride, ref data);

            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            if (data == IntPtr.Zero)
            {
                return img;
            }
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format24bppRgb, data);

            if (tempBitmap.Width == 1)
            {
                return img;
            }

            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);

                BitmapImage bitMapimg = new BitmapImage();
                bitMapimg.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                bitMapimg.StreamSource = ms;
                bitMapimg.CacheOption = BitmapCacheOption.OnLoad;
                bitMapimg.EndInit();
                bitMapimg.Freeze();
                img = bitMapimg;
            }

            return img;
        }

        public static List<CRUXELLLACC.DeviceInfo> DeviceInfos = new List<CRUXELLLACC.DeviceInfo>();

        private static CRUXELLLACC.DeviceInfo? selectDevice;
        public static CRUXELLLACC.DeviceInfo? SelectDevice
        {
            get
            {
                return selectDevice;
            }
            set
            {
                fpga.SelectedDevice = value;
                selectDevice = value;
            }
        }
        public static bool IsFpgaAvailable { get; private set; }
        private static void UpdateDeviceList(object? sender, EventArgs e)
        {
            StatusMsg = "FPGA device list update";
            DeviceInfos = new List<CRUXELLLACC.DeviceInfo>(fpga.DeviceList);
            SelectDevice = fpga.SelectedDevice;
            if (DeviceInfos.Count > 0)
            {
                StatusMsg = "FPGA usb is connected";
                IsFpgaAvailable = true;
            }
            else
            {
                StatusMsg = "FPGA usb is unconnected";
                IsFpgaAvailable = false;
            }
            StatusUpdateInvoke(null, EventArgs.Empty);
        }
        private static bool isSessionStart = false;
        public static bool IsSessionStart
        {
            get { return isSessionStart; }
            private set { StatusUpdateInvoke(null, EventArgs.Empty); isSessionStart = value; }
        }
        private static bool isSessionStarting = false;

        public static bool IsSessionStarting
        {
            get { return isSessionStarting; }
            private set { StatusUpdateInvoke(null, EventArgs.Empty); isSessionStarting = value; }
        }

        public static async Task StartSessionAsync(string fileName, CancellationTokenSource tokenSource)
        {
            IsSessionStarting = true;
            if (!IsInitiate)
            {
                StatusMsg = "LAHGI is not initiated";
            }
            if (!IsSessionStart)
            {
                if (!fpga.SetVaribles(fpgaVariables))
                {
                    StatusMsg = "Please configure FPGA.";
                    IsSessionStarting = false;
                    return;
                }
                else
                {
                    IsSessionStart = true;
                    StatusMsg = "FPGA setting Start";

                    string status = "";

                    bool isFPGAStart = await Task.Run(() => fpga.Start_usb(out status)).ConfigureAwait(false);
                    StatusMsg = status;

                    if (isFPGAStart)
                    {
                        IsSessionStart = true;
                        StartSlam();
                        System.Timers.Timer aTimer = new System.Timers.Timer(100);
                        // Hook up the Elapsed event for the timer. 
                        aTimer.Elapsed += StatusUpdateInvoke;
                        aTimer.Start();
                        StatusUpdateInvoke(null, new LahgiApiEnvetArgs(eLahgiApiEnvetArgsState.Slam));
                        isSessionStarting = false;
                        await Task.Run(() => AddListModeData(tokenSource)).ConfigureAwait(false);

                        IsSessionStarting = false;

                        StatusMsg = await fpga.Stop_usb();
                        await Task.Run(() => StopSlam());
                        aTimer.Stop();
                        aTimer.Elapsed -= StatusUpdateInvoke;
                        StatusMsg = "Saving CSV file";
                        string saveFileName = Path.GetDirectoryName(fpga.FileMainPath) + "\\" + fileName;
                        lahgiWrapper.SaveListModeData(saveFileName + "_LMData.csv");
                        StatusMsg = "Done saving CSV file";
                    }
                    else
                    {
                        IsSessionStart= false;
                        StatusMsg = "Something wrong with FPGA";
                        IsSessionStarting = false;

                        return;
                    }
                    
                }
            }
            else
            {
                StatusMsg = "Session is already started";
                IsSessionStarting = false;

                return;
            }

            StatusMsg = "Session is done";
            IsSessionStarting = false;

            return;
        }
        public record AddListModeDataEchk(double MinE, double MaxE);

        private static bool isEchksChanged = false;
        private static List<AddListModeDataEchk> echks = new List<AddListModeDataEchk>();
        public static List<AddListModeDataEchk> Echks
        {
            get
            {
                return echks;
            }
            set
            {
                isEchksChanged = true;
                echks = Echks;
            }
        }        
        private static void AddListModeData(CancellationTokenSource tokenSource)
        {
            List<double[]> UnmanagedEcks = new List<double[]>();
            foreach (var eck in Echks)
            {
                double[] eckUnmanaged = new double[] { eck.MinE, eck.MaxE };
                UnmanagedEcks.Add(eckUnmanaged);
            }
            lahgiWrapper.ResetListmodeData();
            for (uint i = 0; i < 16; ++i)
            {
                lahgiWrapper.ResetSpectrum(i);
            }

            while (true)
            {
                ushort[] item;
                while (fpga.ShortArrayQueue.TryTake(out item!))
                {
                    lahgiWrapper.AddListModeDataWraper(item, UnmanagedEcks);          
                    if (isEchksChanged)
                    {
                        isEchksChanged = false;
                        UnmanagedEcks.Clear();
                        foreach (var eck in Echks)
                        {
                            double[] eckUnmanaged = new double[] { eck.MinE, eck.MaxE };
                            UnmanagedEcks.Add(eckUnmanaged);
                        }
                    }

                    if (tokenSource.IsCancellationRequested)
                    {
                        break;
                    }
                    Thread.Sleep(0);
                }
                if (tokenSource.IsCancellationRequested)
                {
                    break;
                }
                Thread.Sleep(0);
            }
            StatusMsg = "Add List Mode Data loop ended";
        }
        private static void StartSlam()
        {
            string temp = "";
            rtabmapWrapper.StartSLAM();
            StatusMsg = temp;                        
        }
        private static void StopSlam()
        {
            rtabmapWrapper.StopSLAM();            
        }
        private static Matrix3D currentSystemTranformation = Matrix3D.Identity;
        public static Matrix3D CurrentSystemTranformation
        {
            get 
            {
                double[] marix3DElement = new double[0];
                rtabmapWrapper.GetPoseFrame(ref marix3DElement);
                if (marix3DElement != null)
                {

                    currentSystemTranformation = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                            marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7],
                                            marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11],
                                            marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);
                }
                else
                {
                    currentSystemTranformation = new Matrix3D(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
                }
                SystemPoseX = currentSystemTranformation.OffsetX;
                SystemPoseY = currentSystemTranformation.OffsetY;
                SystemPoseZ = currentSystemTranformation.OffsetZ;
                return currentSystemTranformation; 
            }
            private set
            {
                currentSystemTranformation = value;

            }
        }
        public static double SystemPoseX { get; private set; }
        public static double SystemPoseY { get; private set; }
        public static double SystemPoseZ { get; private set; }
        public static bool GetSLAMPointCloud(ref List<double[]> poseVect, ref List<double[]> colorVect)
        {
            if (!IsSessionStart)
            {
                return false;
            }
            rtabmapWrapper.GetSLAMPointCloud(ref poseVect, ref colorVect);
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }
            return true;
        }
        public static bool GetReconSLAMPointCloud(double time, eReconManaged reconType, ref List<double[]> poseVect, ref List<double[]> colorVect, double voxelSize, bool isLoading)
        {
     
            rtabmapWrapper.GetReconSLAMPointCloud(time, reconType, ref poseVect, ref colorVect, voxelSize, isLoading);
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }
            return true;
        }
        
        public static SpectrumEnergyNasa GetSpectrumEnergy(int channelNumber)
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSpectrum((uint)channelNumber, ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetSumSpectrumEnergy()
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetSumSpectrum(ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetAbsorberSumSpectrum()
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetAbsorberSumSpectrum(ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetScatterSumSpectrum()
        {
            if (!IsLahgiInitiate)
            {
                return new SpectrumEnergyNasa(5, 3000);
            }
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetScatterSumSpectrum(ref eCount);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetScatterSumSpectrumByTime(uint time)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetScatterSumSpectrumByTime(ref eCount, time);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }
        public static SpectrumEnergyNasa GetAbsorberSumSpectrumByTime(uint time)
        {
            List<double[]> eCount = new List<double[]>();
            lahgiWrapper.GetAbsorberSumSpectrumByTime(ref eCount, time);
            List<HistoEnergy> histoEnergy = new List<HistoEnergy>();
            for (int i = 0; i < eCount.Count; i++)
            {
                histoEnergy.Add(new HistoEnergy(eCount[i][0], Convert.ToInt32(eCount[i][1])));
            }
            SpectrumEnergyNasa spect = new SpectrumEnergyNasa(histoEnergy);
            return spect;
        }        

        public static bool LoadListModeData(string filePath)
        {
            return lahgiWrapper.LoadListModeData(filePath);
        }

        public static bool LoadPlyFile(string filePath)
        {
            if (Path.GetExtension(filePath) != ".ply")
            {
                log.Error("LoadPyFile fail due to extenstion is not ply");
                return false;
            }
            log.Info($"Loading ply file: {filePath}");
            if (rtabmapWrapper.LoadPlyFile(filePath))
            {
                log.Info("Loading seccess");
                return true;
            }
            else
            {
                log.Info("Loading fail");
                return false;
            }            
        }

        public static void GetLoadedPointCloud(ref List<double[]> poseVect, ref List<double[]> colorVect)
        {
            rtabmapWrapper.GetLoadedPointCloud(ref poseVect, ref colorVect);            
        }
    }
}
    
