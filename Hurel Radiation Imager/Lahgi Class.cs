using System.Configuration;
using System.Diagnostics;
using System.IO;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;
using Newtonsoft.Json.Linq;


namespace HUREL.Compton
{
    public record AddListModeDataEchk(double MinE, double MaxE);

    public enum eLahgiApiEnvetArgsState
    {
        Loading,
        SlamPoints,
        SlamRadImage,
        Spectrum,
        Massage,
        Status
    }
    public enum eEcalState
    {
        Fail,
        Success,
        Unknown
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

        public static void StatusUpdateInvoke(object? obj, eLahgiApiEnvetArgsState state)
        {
            Task.Run(()=> { StatusUpdate?.Invoke(obj, new LahgiApiEnvetArgs(state)); });

        }

        private static bool timerBoolSlamPoints = false;
        private static bool timerBoolSlamRadImage = false;
        private static bool timerBoolSpectrum = false;
        private static void UpdateTimerInvoker(object? obj, EventArgs args)
        {
            if (timerBoolSlamPoints)
            {
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.SlamPoints);
            }
            if (timerBoolSlamRadImage)
            {
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.SlamRadImage);
            }
            if (timerBoolSpectrum)
            {
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }

        private static string statusMsg = "";
        public static string StatusMsg
        {
            get
            {
                return statusMsg;
            }

            set
            {
                log = LogManager.GetLogger("LahgiApi");
                log.Info(value);
                statusMsg = value;
                Trace.WriteLine(value);
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Massage);
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

            IsSavingBinary = false;
            UpdateDeviceList(null, EventArgs.Empty);

            System.Timers.Timer timer = new System.Timers.Timer();
            timer.Interval = 500;
            timer.Elapsed += UpdateTimerInvoker;
            timer.Start();

            InitialLizeConfigFile();
        }

        private static void InitialLizeConfigFile()
        {
            var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            var appSetting = configFile.AppSettings.Settings;
            if (appSetting["Test"] == null)
            {
                appSetting.Add("Test", "0");
            }
            if (appSetting[nameof(ref_x)] == null)
            {
                appSetting.Add(nameof(ref_x), "662");
            }
            if (appSetting[nameof(ref_fwhm)] == null)
            {
                appSetting.Add(nameof(ref_fwhm), "50");
            }
            if (appSetting[nameof(ref_at_0)] == null)
            {
                appSetting.Add(nameof(ref_at_0), "10");
            }
            if (appSetting[nameof(min_snr)] == null)
            {
                appSetting.Add(nameof(min_snr), "5");
            }
            if (appSetting[nameof(LastBootUp)] == null)
            {
                appSetting.Add(nameof(LastBootUp), DateTime.Now.Ticks.ToString());
            }
            for (int i = 0; i < 8; i++)
            {
                appSetting.Add(nameof(eEcalStates) + i.ToString(), eEcalState.Unknown.ToString());
            }

            configFile.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
        }

        private static float ref_x = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(ref_x)));
        public static float Ref_x
        {
            get { return ref_x; }
            set
            {
                ref_x = value;

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(ref_x)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);

            }
        }

        private static float ref_fwhm = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(ref_fwhm)));
        public static float Ref_fwhm
        {
            get { return ref_fwhm; }
            set
            {
                ref_fwhm = value;

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(ref_fwhm)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }
        private static float ref_at_0 = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(ref_at_0)));
        public static float Ref_at_0
        {
            get { return ref_at_0; }
            set
            {
                ref_at_0 = value;
                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(ref_at_0)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
               }
        }
        private static float min_snr = Convert.ToSingle(ConfigurationManager.AppSettings.Get(nameof(min_snr)));
        public static float Min_snr
        {
            get { return min_snr; }
            set
            {
                min_snr = value;
                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(min_snr)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
            }
        }
        public static readonly DateTime LastBootUp = new DateTime(Convert.ToInt64(ConfigurationManager.AppSettings.Get(nameof(LastBootUp))));

        /// <summary>
        /// Ecal Scatter 4, Absorber 4
        /// </summary>
        public static List<eEcalState> eEcalStates = new List<eEcalState>();

        public static bool InitiateLaghi()
        {
            StatusMsg = "Initiating LAHGI";
            var tempEchk = new List<AddListModeDataEchk>();
            ////tempEchk.Add(new AddListModeDataEchk(30, 90));
            ////tempEchk.Add(new AddListModeDataEchk(60, 100));
            ////tempEchk.Add(new AddListModeDataEchk(330, 370));
            ////tempEchk.Add(new AddListModeDataEchk(450, 570));
            ////tempEchk.Add(new AddListModeDataEchk(1200, 1350));
            
            tempEchk.Add(new AddListModeDataEchk(550, 690));
            //tempEchk.Add(new AddListModeDataEchk(1173 - 70, 1173 + 70));
            //tempEchk.Add(new AddListModeDataEchk(1333 - 50, 1333 + 50));
            Echks = tempEchk;
            if (lahgiWrapper.Initiate(eModuleManagedType.QUAD))
            {
                StatusMsg = "Successfully initiate Lahgi Software";

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(LastBootUp)].Value = DateTime.Now.Ticks.ToString();

                if (LastBootUp < DateTime.Now.AddDays(-1))
                {
                    eEcalStates.Clear();
                    for (int i = 0; i < 8; i++)
                    {
                        eEcalStates.Add(eEcalState.Unknown);
                    }
                }
                else
                {
                    eEcalStates.Clear();
                    for (int i = 0; i < 8; i++)
                    {
                        eEcalStates.Add((eEcalState)Enum.Parse(typeof(eEcalState), appSetting[nameof(eEcalStates) + i.ToString()].Value) );
                    }
                }

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);


                LahgiSerialControl.StartCommunication();

                IsLahgiInitiate = true;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Lahgi";
                IsLahgiInitiate = false;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
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
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Rtabmap";
                IsRtabmapInitiate = false;
                StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                return false;
            }
        }

        public static void StopAll()
        {
            StopSlam();
            rtabmapWrapper.StopVideoStream();
        }

        public static Mutex GetResponseImageMutex = new Mutex();
        public static BitmapImage? GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
        {
            BitmapImage? img = null;
            int width = 1;
            int height = 1;
            int stride = 1;
            GetResponseImageMutex.WaitOne();
            IntPtr data = IntPtr.Zero;
            var outData = lahgiWrapper.GetResponseImage(imgSize, pixelCount, timeInSeconds, isScatter);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            data = outData.ptr;

            if (data == IntPtr.Zero)
            {
                GetResponseImageMutex.ReleaseMutex();
                return img;
            }

            width = outData.width;
            height = outData.height;
            stride = outData.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, data);

            if (tempBitmap.Width == 1)
            {
                GetResponseImageMutex.ReleaseMutex();
                return img;
            }

            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
            }
            GetResponseImageMutex.ReleaseMutex();
            return img;
        }

        public static Mutex GetRadation2dImageMutex = new Mutex();

        public static (BitmapImage?, BitmapImage?, BitmapImage?) GetRadation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion)
        {
            BitmapImage? imgCoded = null;
            BitmapImage? imgCompton = null;
            BitmapImage? imgHybrid = null;

            if (!GetRadation2dImageMutex.WaitOne())
            {
                return (imgCoded, imgCompton, imgHybrid);
            }
            var outData = lahgiWrapper.Get2dRadationImage(timeInMiliSeconds, s2M, det_W, resImprov, m2D, hFov, wFov, imgSize, minValuePortion);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr dataCoded = outData.Item1.ptr;
            IntPtr dataCompton = outData.Item2.ptr;
            IntPtr dataHybrid = outData.Item3.ptr;

            if (dataCompton == IntPtr.Zero || outData.Item1.width == 1)
            {
                GetRadation2dImageMutex.ReleaseMutex();
                return (imgCoded, imgCompton, imgHybrid);
            }

            int width = outData.Item1.width;
            int height = outData.Item1.height;
            int stride = outData.Item1.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCoded);

           
            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCoded = new BitmapImage();
                imgCoded.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCoded.StreamSource = ms;
                imgCoded.CacheOption = BitmapCacheOption.OnLoad;
                imgCoded.EndInit();
                imgCoded.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataCompton);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgCompton = new BitmapImage();
                imgCompton.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgCompton.StreamSource = ms;
                imgCompton.CacheOption = BitmapCacheOption.OnLoad;
                imgCompton.EndInit();
                imgCompton.Freeze();
            }


            tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, dataHybrid);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                imgHybrid = new BitmapImage();
                imgHybrid.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                imgHybrid.StreamSource = ms;
                imgHybrid.CacheOption = BitmapCacheOption.OnLoad;
                imgHybrid.EndInit();
                imgHybrid.Freeze();
            }

            GetRadation2dImageMutex.ReleaseMutex();


            return (imgCoded, imgCompton, imgHybrid);
        }
        public static Mutex GetTransPoseRadiationImageMutex = new Mutex();


        public static BitmapImage? GetTransPoseRadiationImage(int timeInMiliSecond, double minValuePortion, double resolution = 10)
        {
            BitmapImage? img= null;
            
            if (!GetTransPoseRadiationImageMutex.WaitOne())
            {
                return img;
            }
            var outData = lahgiWrapper.GetTransPoseRadiationImage(timeInMiliSecond, minValuePortion, resolution);
            //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
            // Bitmap 담을 메모리스트림 
            IntPtr data = outData.ptr;

            if (data == IntPtr.Zero || outData.width == 1)
            {
                GetRadation2dImageMutex.ReleaseMutex();
                return img;
            }

            int width = outData.width;
            int height = outData.height;
            int stride = outData.stride;
            Bitmap tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format32bppArgb, data);


            using (MemoryStream ms = new MemoryStream())
            {
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Png);
                //tempBitmap.Save("test.png");
                img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
            }

            GetTransPoseRadiationImageMutex.ReleaseMutex();


            return img;
        }
        public static BitmapImage? GetRgbImage()
        {

            if (!IsRtabmapInitiate)
            {
                return null;
            }
            BitmapImage? img = null;
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

                img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
                //img = bitMapimg;
            }
            

            //tempBitmap.Dispose();

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
            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
        }

        static public Stopwatch SessionStopwatch = new Stopwatch();
        private static bool isSessionStart = false;
        public static bool IsSessionStart
        {
            get { return isSessionStart; }
            private set { StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status); isSessionStart = value; }
        }
        private static bool isSessionStarting = false;

        public static bool IsSessionStarting
        {
            get { return isSessionStarting; }
            set { StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status); isSessionStarting = value; }
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
                    fpga.Variables.FileName = fileName;
                    bool isFPGAStart = await Task.Run(() => fpga.Start_usb(out status)).ConfigureAwait(false);
                    StatusMsg = status;

                    if (isFPGAStart)
                    {
                        IsSessionStart = true;
                        StartSlam();

                        StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                        isSessionStarting = false;
                        timerBoolSpectrum = true;
                        timerBoolSlamRadImage = true;
                        SessionStopwatch.Restart();
                        await Task.Run(() => AddListModeData(tokenSource)).ConfigureAwait(false);
                        SessionStopwatch.Stop();

                        timerBoolSpectrum = false;
                        timerBoolSlamRadImage = false;
                        IsSessionStarting = true;
                        StatusMsg = "Stopping usb";

                        StatusMsg = await fpga.Stop_usb();
                        IsSessionStarting = true;
                        StatusMsg = "Saving CSV and ply file";

                        string saveFileName = Path.GetDirectoryName(fpga.FileMainPath) + "\\" + fileName;
                        rtabmapWrapper.SavePlyFile(saveFileName + "_SlamData.ply");

                        lahgiWrapper.SaveListModeData(saveFileName);
                        StatusMsg = "Done saving CSV and ply file";

                        await Task.Run(() => StopSlam());
               

                        IsSessionStarting = false;
                        IsSessionStart = false;
                    }
                    else
                    {
                        IsSessionStart = false;
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




        private static List<AddListModeDataEchk> echks = new List<AddListModeDataEchk>();
        public static List<AddListModeDataEchk> Echks
        {
            get
            {
                return echks;
            }
            set
            {

                List<double[]> UnmanagedEcks = new List<double[]>();

                UnmanagedEcks.Clear();
                foreach (var eck in value)
                {
                    double[] eckUnmanaged = new double[] { eck.MinE, eck.MaxE };
                    UnmanagedEcks.Add(eckUnmanaged);
                }
                lahgiWrapper.SetEchks(UnmanagedEcks);

                echks = value;
            }
        }
        private static void AddListModeData(CancellationTokenSource tokenSource)
        {          
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
                    lahgiWrapper.AddListModeDataWraper(item);
                   
                    if (tokenSource.IsCancellationRequested)
                    {
                        break;
                    }
                }
                if (tokenSource.IsCancellationRequested)
                {
                    break;
                }
                //Thread.Sleep(0);
            }
            IsSessionStarting = true;

            while (fpga.ShortArrayQueue.Count > 0)
            {
                ushort[] item;
                fpga.ShortArrayQueue.TryTake(out item);
            }
            StatusMsg = "Add List Mode Data loop ended";
        }

        public static void TestAddingListModeData(int count)
        {
            StatusMsg = $"TestAddingListModeData starts (count = {count})";

            Random rnd = new Random();
            for (int c = 0; c < count; ++c)
            {
                ushort[] shortArray = new ushort[144];

                for (int j = 0; j < 144; ++j)
                {
                    shortArray[j] = 0;
                }
                //Channel 4 to 8
                for (int i = 4; i < 5; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        shortArray[i * 9 + j] = (ushort)rnd.Next(300);
                    }
                }

                //Channel 12 to 16
                for (int i = 12; i < 13; ++i)
                {
                    for (int j = 0; j < 9; ++j)
                    {
                        shortArray[i * 9 + j] = (ushort)rnd.Next(300);
                    }
                }
                fpga.ShortArrayQueue.Add(shortArray);
            }
            lahgiWrapper.ResetListmodeData();
            for (uint i = 0; i < 16; ++i)
            {
                lahgiWrapper.ResetSpectrum(i);
            }

            StatusMsg = $"TestAddingListModeData loop starts (count = {count})";
            Stopwatch sw = Stopwatch.StartNew();
            ushort[] item;
            while (fpga.ShortArrayQueue.TryTake(out item!))
            {
                lahgiWrapper.AddListModeDataWraper(item);
                //Thread.Sleep(0);
            }
            while (lahgiWrapper.GetListedListModeDataSize() != count)
            {

            }
            
            sw.Stop();
            Thread.Sleep(0);

            GetResponseImage(500, 200, 0, true);

            StatusMsg = $"TestAddingListModeData took {sw.ElapsedMilliseconds} ms for {count} counts";
        }

        public static void StartSlam()
        {
            rtabmapWrapper.StartSLAM();
            timerBoolSlamPoints = true;
            StatusMsg = "Slam Started";
        }
        public static void StopSlam()
        {
            timerBoolSlamPoints = false;
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

        public static bool GetOptimizedPoses(ref List<Matrix3D> matrixes)
        {
            matrixes = new List<Matrix3D>();
            List<double[]> posesMat = new List<double[]>();

            rtabmapWrapper.GetOptimizePoses(ref posesMat);
            if (posesMat.Count == 0)
            {
                return false;
            }

            foreach (var marix3DElement in posesMat)
            {
                Matrix3D pose = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                            marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7],
                                            marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11],
                                            marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);
                matrixes.Add(pose);
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
                return new SpectrumEnergyNasa(10, 3000);
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
                return new SpectrumEnergyNasa(10, 3000);
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

        public static bool Ecal609keV = false;

        public static void CheckEcalState(double min1461E = 1300, double max1461E = 1500)
        {
            StatusMsg = "Ecal Start";
            for (int i =0; i < 8; ++i)
            {
                if (eEcalStates[i] == eEcalState.Success)
                {
                    if (i == 7)
                    {
                        return;
                    }
                }
                else
                {
                    break;
                }
            }
            for (int i =0; i <4; ++i)
            {
                //Scatter

                StatusMsg = "Ecal " + i + 4 + " Channel";

                var peaks = GetSpectrumEnergy(i + 4).FindPeaks(ref_x, ref_fwhm, ref_at_0, min_snr);
                bool isEcalSuccessFlag = false;
                foreach(var e in peaks)
                {
                    if (Ecal609keV)
                    {

                    }
                    else
                    {
                        if (e >= min1461E && e <= max1461E)
                        {
                            double ecalA = 0;
                            double ecalB = 0;
                            double ecalC = 0;
                            lahgiWrapper.GetEcal(Convert.ToUInt32(i + 4), ref ecalA, ref ecalB, ref ecalC) ;
                            //Only linearity affect
                            ecalA *= 1461 * 1461 / e / e;
                            ecalB *= 1461 / e;
                            
                            lahgiWrapper.SetEcal(Convert.ToUInt32(i + 4), ecalA, ecalB, ecalC);
                            
                            eEcalStates[i] = eEcalState.Success;
                            isEcalSuccessFlag = true;
                        }

                    }
                    if (!isEcalSuccessFlag)
                    {
                        eEcalStates[i] = eEcalState.Fail;
                    }
                }               
            }
            for (int i = 0; i < 4; ++i)
            {
                //Absorber
                StatusMsg = "Ecal " + i + 12 + " Channel";

                var peaks = GetSpectrumEnergy(i + 12).FindPeaks(ref_x, ref_fwhm, ref_at_0, min_snr);
                bool isEcalSuccessFlag = false;

                foreach (var e in peaks)
                {
                    if (Ecal609keV)
                    {

                    }
                    else
                    {
                        if (e >= min1461E && e <= max1461E)
                        {
                            double ecalA = 0;
                            double ecalB = 0;
                            double ecalC = 0;
                            lahgiWrapper.GetEcal(Convert.ToUInt32(i + 12), ref ecalA, ref ecalB, ref ecalC);
                            //Only linearity affect
                            ecalA *= 1461 * 1461 / e / e;
                            ecalB *= 1461 / e;

                            lahgiWrapper.SetEcal(Convert.ToUInt32(i + 12), ecalA, ecalB, ecalC);

                            eEcalStates[i + 4] = eEcalState.Success;
                            isEcalSuccessFlag = true;
                        }
                    }
                    if (!isEcalSuccessFlag)
                    {
                        eEcalStates[i + 4] = eEcalState.Fail;
                    }
                }
               
            }
            StatusMsg = "Ecal Done";



            var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            var appSetting = configFile.AppSettings.Settings;
            
            for (int i = 0; i < 8; i++)
            {
                appSetting[nameof(eEcalStates) + i.ToString()].Value = eEcalState.Unknown.ToString();
            }
            configFile.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
            LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);


            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
        }

        public static bool LoadListModeData(string filePath)
        {
            lahgiWrapper.ResetListmodeData();
            for (uint i = 0; i < 16; ++i)
            {
                lahgiWrapper.ResetSpectrum(i);
            }
            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);
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
            rtabmapWrapper.ResetSLAM();
            StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Loading);

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
    
