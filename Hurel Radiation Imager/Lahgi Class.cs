using System.IO;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using log4net;

[assembly: log4net.Config.XmlConfigurator(Watch = true)]
namespace HUREL.Compton
{
    public static class LahgiApi
    {
        private static ILog log = LogManager.GetLogger("LahgiApi");

        private static CRUXELLLACC fpga;
        private static LahgiWrapper lahgiWrapper;
        private static RtabmapWrapper rtabmapWrapper;
        public static CRUXELLLACC.VariableInfo fpgaVariables;

        private static string statusMsg = "";
        public static string StatusMsg
        {
            get
            {
                return statusMsg;
            }
       
            private set
            {
                log.Info(value);
                statusMsg = value;                
            }
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
            InitiateLaghi();
            InititateRtabmap();
        }        
        public static bool InitiateLaghi()
        {
            StatusMsg = "Initiating LAHGI";
            
            if (lahgiWrapper.Initiate(eModuleManagedType.QUAD))
            {
                StatusMsg = "Successfully initiate Lahgi";
                IsLahgiInitiate = true;
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Lahgi";
                IsLahgiInitiate = false;
                return false;
            }
        }
        public static bool InititateRtabmap()
        {

            StatusMsg = "Initiating RTABAMP";

            string msg = "";
            if (rtabmapWrapper.InitiateRtabmap(ref msg))
            {
                StatusMsg = "Successfully initiate Rtabmap";
                StatusMsg = msg;
                IsRtabmapInitiate = true;
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Rtabmap";
                StatusMsg = msg;
                IsRtabmapInitiate = false;
                return false;
            }
        }
        public static bool StartRtabmap()
        {
            if (!IsRtabmapInitiate)
            {
                return false;
            }
            string msg = "";
            bool isSuccess = rtabmapWrapper.StartRtabmapPipeline(ref msg);
            StatusMsg = msg;
            return isSuccess;
        }
        public static void StopRtabmap()
        {
            rtabmapWrapper.StopRtabmapPipeline();
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
        }
        private static bool isSessionStart = false;
        public static bool IsSessionStart
        {
            get { return isSessionStart; }
            private set { isSessionStart = value; }
        }    
        public static async Task StartSessionAsync(string fileName, int measurementTime, CancellationTokenSource tokenSource)
        {
            if (!IsSessionStart)
            {
                if (!fpga.SetVaribles(fpgaVariables))
                {
                    StatusMsg = "Please configure FPGA.";
                    return;
                }
                else
                {
                    StatusMsg = "FPGA setting Start";

                    string status = "";

                    bool isFPGAStart = await Task.Run(() => fpga.Start_usb(out status)).ConfigureAwait(false);
                    StatusMsg = status;

                    if (isFPGAStart)
                    {
                        IsSessionStart = true;
                        StartSlam();
                        await Task.Run(() => AddListModeData(tokenSource));

                        IsSessionStart = false;

                        StatusMsg = await fpga.Stop_usb();
                        await Task.Run(() => StopSlam());


                        StatusMsg = "Saving CSV file";
                        string saveFileName = Path.GetDirectoryName(fpga.FileMainPath) + "\\" + fileName;
                        lahgiWrapper.SaveListModeData(saveFileName + "_LMData.csv");
                        StatusMsg = "Done saving CSV file";


                    }
                    
                }
            }
            else
            {
                StatusMsg = "Session is already started";
            }
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
                }
                if (tokenSource.IsCancellationRequested)
                {
                    break;
                }
            }
            StatusMsg = "Add List Mode Data loop ended";
        }
        private static void StartSlam()
        {
            string temp = "";
            rtabmapWrapper.StartSLAM(ref temp);
            StatusMsg = temp;
            rtabmapWrapper.ResetPipeline();                        
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
        public static bool GetReconSLAMPointCloud(double time, eReconType reconType, ref List<double[]> poseVect, ref List<double[]> colorVect)
        {
            if (!IsSessionStart)
            {
                return false;
            }
            rtabmapWrapper.GetReconSLAMPointCloud(time, reconType, ref poseVect, ref colorVect, 0.1);
            if (poseVect.Count == 0 || colorVect.Count == 0)
            {
                return false;
            }
            return true;
        }

    }
}
    
