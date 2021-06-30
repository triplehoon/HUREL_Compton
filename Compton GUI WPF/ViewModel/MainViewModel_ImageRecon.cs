using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using HUREL.Compton.LACC;
using System.Diagnostics;
using GalaSoft.MvvmLight.Command;
using System.Windows.Input;
using HUREL.Compton;
using System.Windows.Media.Media3D;
using System.Windows.Media.Imaging;
using System.Drawing;
using System.IO;
using AsyncAwaitBestPractices.MVVM;
using System.Windows;
using System.Windows.Threading;
using System.Windows.Interop;
using System.Collections.ObjectModel;
using System.Drawing.Imaging;

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel
    {
        private RealsenseControlWrapper RealsenseControl = new RealsenseControlWrapper();

        private Vector3D T265ToLACCOffset = new Vector3D(0, -0.308, -0.05);
        private double t265ToLACCOffsetX;
        public double T265ToLACCOffsetX
        {
            get { return t265ToLACCOffsetX; }
            set
            {
                T265ToLACCOffset.X = value;
                t265ToLACCOffsetX = value;
                OnPropertyChanged(nameof(t265ToLACCOffsetX));
            }
        }

        private double t265ToLACCOffsetY;
        public double T265ToLACCOffsetY
        {
            get { return t265ToLACCOffsetY; }
            set
            {
                T265ToLACCOffset.Y = value;
                t265ToLACCOffsetY = value;
                OnPropertyChanged(nameof(t265ToLACCOffsetY));
            }
        }

        private double t265ToLACCOffsetZ;
        public double T265ToLACCOffsetZ
        {
            get { return t265ToLACCOffsetZ; }
            set
            {
                T265ToLACCOffset.Z = value;
                t265ToLACCOffsetZ = value;
                OnPropertyChanged(nameof(t265ToLACCOffsetZ));
            }
        }


        private string realsenseState;
        public string RealsenseState
        {
            get { return realsenseState; }
            set { realsenseState = value; OnPropertyChanged(nameof(RealsenseState)); }
        }

        private bool isRealsenseOn;
        public bool IsRealsenseOn
        {
            get { return isRealsenseOn; }
            set { isRealsenseOn = value; OnPropertyChanged(nameof(IsRealsenseOn)); }
        }

        private AsyncCommand initiateRealsenseCommand;
        public ICommand InitiateRealsenseCommand
        {
            get { return (this.initiateRealsenseCommand) ?? (this.initiateRealsenseCommand = new AsyncCommand(InitiateRealsenseAsync)); }
        }
        private async Task InitiateRealsenseAsync()
        {
            RealsenseState = "Initiating Realsense";
            await Task.Run(() =>
            {
                string s = "";
                IsRealsenseOn = RealsenseControl.InitiateRealsense(ref s);
                RealsenseState = s;
                
            });
            await StartRealsensePipeline();
        }

        #region Realsense Pipeline Setup

        private bool isRealsensePipelineOn;
        public bool IsRealsensePipelineOn
        {
            get { return isRealsensePipelineOn; }
            set 
            { 
                isRealsensePipelineOn = value; 
                OnPropertyChanged(nameof(IsRealsensePipelineOn));
                Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        ((AsyncCommand)StartSLAMCommand).RaiseCanExecuteChanged();
                        ((AsyncCommand)StopSLAMCommand).RaiseCanExecuteChanged();
                    }));
                
            }
        }

        private AsyncCommand startRealsensePipelineCommand;
        public ICommand StartRealsensePipelineCommand
        {
            get 
            { 
                return (this.startRealsensePipelineCommand) ?? (this.startRealsensePipelineCommand = new AsyncCommand(StartRealsensePipeline)); 
            }
        }

        private async Task StartRealsensePipeline()
        {
            await Task.Run(() =>
            {
                string s = "";
                if (!IsRealsenseOn)
                {
                    RealsenseState = "Please Initiate Realsense.";
                    return;
                }
                IsRealsensePipelineOn = RealsenseControl.StartRealsensePipeline(ref s);
                if (IsRealsensePipelineOn)
                {
                    UpdateRGBPoseTask = Task.Run(() => UpdateRGBPose());
                    UpdateRealTimePointCloudTask = Task.Run(() => UpdateRealTimePointCloud());
                    //UpdateRGBImageTask.Start();
                }
                RealsenseState = s;
            }).ConfigureAwait(false);
            RealsenseState = "Pipeline has started";
        }

        private AsyncCommand resetRealsensePipelineCommand;
        public ICommand ResetRealsensePipelineCommand
        {
            get { return (this.resetRealsensePipelineCommand) ?? (this.resetRealsensePipelineCommand = new AsyncCommand(ResetRealsensePipeline)); }
        }

        private async Task ResetRealsensePipeline()
        {
            await Task.Run(() =>
            {
                string temp = "";
                RealsenseControl.ResetPipeline(ref temp);
                RealsenseState = temp;
            });

        }

        private AsyncCommand stopRealsensePipelineCommand;
        public ICommand StopRealsensePipelineCommand
        {
            get { return (this.stopRealsensePipelineCommand) ?? (this.stopRealsensePipelineCommand = new AsyncCommand(StopRealsensePipeline)); }
        }
        private async Task StopRealsensePipeline()
        {
            if (!IsRealsensePipelineOn)
            {
                RealsenseState = "Pipeline is not on";
                return;
            }
            RealsenseControl.StopRealsensePipeline();
            IsRealsensePipelineOn = false;
            await UpdateRGBPoseTask;
            await UpdateRealTimePointCloudTask;
        }

        private BitmapImage realtimeRGB;
        public BitmapImage RealtimeRGB
        {
            get { return realtimeRGB; }
            set
            {
                realtimeRGB = value;
                OnPropertyChanged(nameof(RealtimeRGB));
            }
        }

        private Task UpdateRGBPoseTask;
        private void UpdateRGBPose()
        {
            int tmpT265TrackingConfidence = 0;
            Thread.Sleep(0);
            while (IsRealsensePipelineOn)
            {
                var marix3DElement = RealsenseControl.GetPoseFrame(ref tmpT265TrackingConfidence);
                if (marix3DElement != null)
                {
                   
                    CurrentSystemTranformation = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                            marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7],
                                            marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11],
                                            marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);                    
                }
                else
                {                    
                    CurrentSystemTranformation = new Matrix3D(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
                }

                T265TrackingConfidence = tmpT265TrackingConfidence;
                if (tmpT265TrackingConfidence == 3)
                {
                    IsT265TrackingConfidence3 = true;
                }
                else
                {
                    IsT265TrackingConfidence3 = false;
                }

                Bitmap tempBitmap = new Bitmap(1, 1);
                int width = 1;
                int height = 1;
                int stride = 1;
                IntPtr data = IntPtr.Zero;
                RealsenseControl.GetRealTimeRGB(ref width, ref height, ref stride, ref data);
                
                //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
                // Bitmap 담을 메모리스트림 
                if (data == IntPtr.Zero)
                {
                    continue;
                }
                tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format24bppRgb, data);
                //tempBitmap = new Bitmap(width, height, stride, System.Drawing.Imaging.PixelFormat.Format16bppGrayScale, data);

                //for (int x = 0; x < tempBitmap.Width; ++x)
                //{
                //    for (int y = 0; y < tempBitmap.Height; ++y)
                //    {
                //        System.Drawing.Color tmpColor = tempBitmap.GetPixel(x, y);
                //        int brightness = (int)(0.299 * tmpColor.R + 0.587 * tmpColor.G + 0.114 * tmpColor.B);
                //        System.Drawing.Color grey = System.Drawing.Color.FromArgb(brightness, brightness, brightness);
                //        tempBitmap.SetPixel(x, y, grey);
                //    }
                //}

                if (tempBitmap.Width == 1)
                {
                    continue;
                }
                //to gray scale
                //tempBitmap = MakeGrayscale3(tempBitmap);
                MemoryStream ms = new MemoryStream();
                //System.Windows.Forms.ToolStripRenderer.CreateDisabledImage(tempBitmap)
                tempBitmap.Save(ms, System.Drawing.Imaging.ImageFormat.Bmp);
                BitmapImage img = new BitmapImage();
                img.BeginInit();
                ms.Seek(0, SeekOrigin.Begin);
                img.StreamSource = ms;
                img.CacheOption = BitmapCacheOption.OnLoad;
                img.EndInit();
                img.Freeze();
                //Debug.WriteLine("Img Update");



                RealtimeRGB = img;

                tempBitmap.Dispose();

            }
        }

        public static Bitmap MakeGrayscale3(Bitmap original)
        {
            //create a blank bitmap the same size as original
            Bitmap newBitmap = new Bitmap(original.Width, original.Height);

            //get a graphics object from the new image
            using (Graphics g = Graphics.FromImage(newBitmap))
            {

                //create the grayscale ColorMatrix
                ColorMatrix colorMatrix = new ColorMatrix(
                   new float[][]
                   {
             new float[] {.3f, .3f, .3f, 0, 0},
             new float[] {.59f, .59f, .59f, 0, 0},
             new float[] {.11f, .11f, .11f, 0, 0},
             new float[] {0, 0, 0, 1, 0},
             new float[] {0, 0, 0, 0, 1}
                   });

                //create some image attributes
                using (ImageAttributes attributes = new ImageAttributes())
                {

                    //set the color matrix attribute
                    attributes.SetColorMatrix(colorMatrix);

                    //draw the original image on the new image
                    //using the grayscale color matrix
                    g.DrawImage(original, new System.Drawing.Rectangle(0, 0, original.Width, original.Height),
                                0, 0, original.Width, original.Height, GraphicsUnit.Pixel, attributes);
                }
            }
            return newBitmap;
        }

        private Bitmap MergedBitmaps(Bitmap bmp1Large, Bitmap bmp2Small)
        {
            System.Drawing.Size resize = new System.Drawing.Size(bmp1Large.Width, bmp1Large.Height);
            Bitmap resizeImage = new Bitmap(bmp2Small, resize);

            Bitmap result = new Bitmap(bmp1Large.Width,
                                       bmp1Large.Height);
         
            using (Graphics g = Graphics.FromImage(result))
            {
                g.DrawImage(bmp1Large, System.Drawing.Point.Empty);
                g.DrawImage(bmp2Small, System.Drawing.Point.Empty);
                g.Dispose();
            }
            return result;
        }
        #region Bitmap converter
        private Bitmap BitmapImage2Bitmap(BitmapImage bitmapImage)
        {
            // BitmapImage bitmapImage = new BitmapImage(new Uri("../Images/test.png", UriKind.Relative));

            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(bitmapImage));
                enc.Save(outStream);
                System.Drawing.Bitmap bitmap = new System.Drawing.Bitmap(outStream);

                return new Bitmap(bitmap);
            }
        }


        public BitmapImage Bitmap2BitmapImage(Bitmap bitmap)
        {
            using (var memory = new MemoryStream())
            {
                bitmap.Save(memory, System.Drawing.Imaging.ImageFormat.Png);
                memory.Position = 0;

                var bitmapImage = new BitmapImage();
                bitmapImage.BeginInit();
                bitmapImage.StreamSource = memory;
                bitmapImage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapImage.EndInit();
                bitmapImage.Freeze();

                return bitmapImage;
            }
        }


        #endregion


        private Matrix3D currentSystemTranformation = Matrix3D.Identity;
        private Matrix3D CurrentSystemTranformation
        {
            get { return currentSystemTranformation; }
            set
            {
                currentSystemTranformation = value;
                SystemPoseX = value.OffsetX;
                SystemPoseY = value.OffsetY;
                SystemPoseZ = value.OffsetZ;
            }
        }

        private double systemPoseX;
        public double SystemPoseX
        {
            get { return systemPoseX; }
            set { systemPoseX = value; OnPropertyChanged(nameof(SystemPoseX)); }
        }

        private double systemPoseY;
        public double SystemPoseY
        {
            get { return systemPoseY; }
            set { systemPoseY = value; OnPropertyChanged(nameof(SystemPoseY)); }
        }

        private double systemPoseZ;
        public double SystemPoseZ
        {
            get { return systemPoseZ; }
            set { systemPoseZ = value; OnPropertyChanged(nameof(SystemPoseZ)); }
        }

        private int t265TrackingConfidence;
        public int T265TrackingConfidence
        {
            get { return t265TrackingConfidence; }
            set { t265TrackingConfidence = value; OnPropertyChanged(nameof(T265TrackingConfidence)); }
        }


        private bool isT265TrackingConfidence3;
        public bool IsT265TrackingConfidence3
        {
            get { return isT265TrackingConfidence3; }
            set { isT265TrackingConfidence3 = value; OnPropertyChanged(nameof(IsT265TrackingConfidence3)); }
        }
        #endregion

        #region Pointcloud
        private List<Vector3> bpVectors = new List<Vector3>();
        private List<Color4> bpColor4s = new List<Color4>();

        private PointGeometry3D rtPointCloud;
        public PointGeometry3D RTPointCloud
        {
            get { return rtPointCloud; }
            set { rtPointCloud = value; OnPropertyChanged(nameof(RTPointCloud)); }
        }

        private double averagePointCloudDepth;
        public double AveragePointCloudDepth
        {
            get
            {
                return averagePointCloudDepth;
            }
            set
            {
                averagePointCloudDepth = value;
                OnPropertyChanged(nameof(AveragePointCloudDepth));
            }
        }

        private Task UpdateRealTimePointCloudTask;
        public void UpdateRealTimePointCloud()
        {
            int error = 0;
            Thread.Sleep(2000);
            while (IsRealsensePipelineOn)
            {
               try
                {
                    AveragePointCloudDepth = RealsenseControl.AverageDepth;
                    Thread.Sleep(0);

                    //RTPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

                }
                catch
                {
                    error++;
                    Trace.WriteLine("Error Count is " + error);
                }
            }
        }

        private void SaveCurrentPointCloud(string path, string fileName)
        {
            string csvPath = Path.Combine(path.ToString(), DateTime.Now.ToString("yyyyMMddHHmm") + "_" + fileName + "_PointCloud.csv");
            string bmpPath = Path.Combine(path.ToString(), DateTime.Now.ToString("yyyyMMddHHmm") + "_" + fileName + "_image.bmp");
            if (RealtimeVector3s == null)
            {
                return;
            }
            SharpDX.Vector3[] tempVector = RealtimeVector3s.ToArray();
            var tempUVs = RealtimeUVs.ToArray();
            var colors = RealtimeCC.ToArray();

            int length = tempVector.Length;

            if (length > colors.Length)
            {
                length = colors.Length;
            }


            var rtBitmap = BitmapImage2Bitmap(RealtimeRGB);
            rtBitmap.Save(bmpPath);

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(csvPath))
            {
                //file.WriteLine("Time[HHMMssFFF],SCposX[m],SCposY,SCposZ,SCEnergy[keV],ABposX,ABposY,ABposZ,ABEnergy");

                for(int i = 0; i < length; ++i)
                {
                    file.WriteLine($"{tempVector[i].X},{tempVector[i].Y},{tempVector[i].Z},{colors[i].Red},{colors[i].Green},{colors[i].Blue},{tempUVs[i][0]},{tempUVs[i][1]}");
                }
            }
        }


        /// <summary>
        /// SLAM
        /// </summary>
        private PointGeometry3D slamPointCloud;
        public PointGeometry3D SLAMPointCloud
        {
            get { return slamPointCloud; }
            set { slamPointCloud = value; OnPropertyChanged(nameof(SLAMPointCloud)); }
        }

        private LineGeometry3D slamPoseInfo;
        public LineGeometry3D SLAMPoseInfo
        {
            get { return slamPoseInfo; }
            set { slamPoseInfo = value; OnPropertyChanged(nameof(SLAMPoseInfo)); }
        }

        private int slamPointCloudCount;
        public int SLAMPointCloudCount
        {
            get { return slamPointCloudCount; }
            set { slamPointCloudCount = value; OnPropertyChanged(nameof(SLAMPointCloudCount)); }
        }

        private bool _isSLAMOn = false;
        private bool IsSLAMOn
        {
            get
            {
                return _isSLAMOn;
            }
            set
            {
                _isSLAMOn = value;
                if (Application.Current == null)
                {
                    return;
                }
                Application.Current.Dispatcher.Invoke(
                    DispatcherPriority.ApplicationIdle,
                    new Action(() => {
                        ((AsyncCommand)StartSLAMCommand).RaiseCanExecuteChanged();
                        ((AsyncCommand)StopSLAMCommand).RaiseCanExecuteChanged();
                    }));
            }
        }

        private AsyncCommand startSLAMCommand;
        public ICommand StartSLAMCommand
        {
            get { return (this.startSLAMCommand) ?? (this.startSLAMCommand = new AsyncCommand(StartSLAM,CanStartSLAMCommand)); }
        }
        private bool CanStartSLAMCommand(object obj)
        {
            bool checkAll = !IsSLAMOn && IsRealsensePipelineOn;
            return checkAll;
        }
        private async Task StartSLAM()
        {
            IsSLAMOn = true;
            string temp = "";
            await Task.Run(()=>RealsenseControl.StartSLAM(ref temp)).ConfigureAwait(false);
            RealsenseState = temp;
            SLAMPointCloud = new PointGeometry3D();
            SLAMPoseInfo = new LineGeometry3D();
            UpdateSLAMPointCloudTask = Task.Run(() => UpdateSLAMPointCloud());
            SLAMReconTaskAsync = Task.Run(() => SLAMRecon());
        }


        private AsyncCommand stopSLAMCommand;
        public ICommand StopSLAMCommand
        {
            get { return (this.stopSLAMCommand) ?? (this.stopSLAMCommand = new AsyncCommand(StopSLAM,CanStopSLAMCommand)); }
        }
        private bool CanStopSLAMCommand(object obj)
        {
            return IsSLAMOn;
        }
        private async Task StopSLAM()
        {
            await Task.Run(() => RealsenseControl.StopSLAM());
            IsSLAMOn = false;
            if (UpdateSLAMPointCloudTask != null) {
                await UpdateSLAMPointCloudTask;
            }
            if(SLAMReconTaskAsync != null) {
                await SLAMReconTaskAsync;
            }
            //SLAMReconTask.Wait();
        }

        private Task UpdateSLAMPointCloudTask;
        public void UpdateSLAMPointCloud()
        {           
            var line = new LineBuilder();
            Vector3 previousPose = new Vector3(Convert.ToSingle(systemPoseX), Convert.ToSingle(systemPoseY), Convert.ToSingle(systemPoseZ));
           

            var poseVect = new List<double[]>();
            var colorVect = new List<double[]>();
            Point3D lineVect = new Point3D(0, 0, 0.3);
            while (IsSLAMOn)
            {
                Thread.Sleep(500);
                var vc = new Vector3Collection();               
                var cc = new Color4Collection();
                poseVect = new List<double[]>();
                colorVect = new List<double[]>();
                Vector3 currentPose = new Vector3(Convert.ToSingle(systemPoseX), Convert.ToSingle(systemPoseY), Convert.ToSingle(systemPoseZ));

                line.AddLine(previousPose, currentPose);

                SLAMPoseInfo = line.ToLineGeometry3D();
                previousPose = currentPose;


                RealsenseControl.GetSLAMPointCloud(ref poseVect, ref colorVect);

                for (int i = 0; i < poseVect.Count; i++)
                {
                    vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                    //cc.Add(new Color4(0.1f, 0.1f, 0.1f, 0.5f));
                    cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.5f));
                    //id.Add(i);
                }
                for (int i = 0; i < 10; ++i)
                {
                    var pointx = new Point3D(0.1 * i, 0, 0);
                    var pointy = new Point3D(0, 0.1 * i, 0);
                    var pointz = new Point3D(0, 0, 0.1 * i);
                    vc.Add(new Vector3(Convert.ToSingle(pointx.X), Convert.ToSingle(pointx.Y), Convert.ToSingle(pointx.Z)));
                    cc.Add(new Color4(1f, 0f, 0f, 1f));
                    vc.Add(new Vector3(Convert.ToSingle(pointy.X), Convert.ToSingle(pointy.Y), Convert.ToSingle(pointy.Z)));
                    cc.Add(new Color4(0f, 1f, 0f, 1f));
                    vc.Add(new Vector3(Convert.ToSingle(pointz.X), Convert.ToSingle(pointz.Y), Convert.ToSingle(pointz.Z)));
                    cc.Add(new Color4(0f, 0f, 1f, 1f));
                }


                for (int i = 0; i < 10; ++i)
                {
                    var pointx = currentSystemTranformation.Transform(new Point3D(0.1 * i, 0, 0));
                    var pointy = currentSystemTranformation.Transform(new Point3D(0, 0.1 * i, 0));
                    var pointz = currentSystemTranformation.Transform(new Point3D(0, 0, 0.1 * i));
                    vc.Add(new Vector3(Convert.ToSingle(pointx.X), Convert.ToSingle(pointx.Y), Convert.ToSingle(pointx.Z)));
                    cc.Add(new Color4(1f, 0f, 0f, 1f));
                    vc.Add(new Vector3(Convert.ToSingle(pointy.X), Convert.ToSingle(pointy.Y), Convert.ToSingle(pointy.Z)));
                    cc.Add(new Color4(0f, 1f, 0f, 1f));
                    vc.Add(new Vector3(Convert.ToSingle(pointz.X), Convert.ToSingle(pointz.Y), Convert.ToSingle(pointz.Z)));
                    cc.Add(new Color4(0f, 0f, 1f, 1f));
                }
                
                SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };
                
                SLAMVector3s = vc;



            }
            Debug.WriteLine("SLAM Point Cloud Count is " + poseVect.Count);
        }


        #endregion

        #region ImageRecon
        private bool isRealTimeImageReconOn = true;
        public bool IsRealTimeImageReconOn
        {
            get { return isRealTimeImageReconOn; }
            set { isRealTimeImageReconOn = value; OnPropertyChanged(nameof(IsRealTimeImageReconOn)); }
        }
        private int mlpeTime;
        public int MLPETime
        {
            get 
            { 
                return mlpeTime; 
            }
            set 
            {
                mlpeTime = value; 
                OnPropertyChanged(nameof(MLPETime)); 
            }
        }
        private int ReconRGBPixelWidth = 848;
        private int ReconRGBPixelHeight = 480;

        public enum RTReconMode
        {
            RTRECONMODE_FAR,
            RTRECONMODE_NEAR
        }
        private RTReconMode realtimeReconMode = RTReconMode.RTRECONMODE_NEAR;
        public RTReconMode RealtimeReconMode
        {
            get { return realtimeReconMode; }
            set 
            { 
                realtimeReconMode = value; 
                OnPropertyChanged(nameof(RealtimeReconMode)); 
            }
        }

        private Task RealTimeImageReconTaskAsync;
        private void RealTimeImageRecon() 
        {
            if (RealtimeRGB != null)
            {
                ReconRGBPixelWidth = RealtimeRGB.PixelWidth  / 10;
                ReconRGBPixelHeight = RealtimeRGB.PixelHeight / 10 ;
            }
            if (!IsRealsenseOn)
            {
                return;
            }



            (SurfaceImageVector3, SurfaceImageUVs) = ImageRecon.GetImageSpaceBySurfaceFOV(ReconRGBPixelWidth, ReconRGBPixelHeight, 90, 65, 10);
            while (IsRealTimeImageReconOn)
            {
                //Stopwatch sw = new Stopwatch();
                //sw.Start();
                ////DrawBPPointCloudToRealTimePointCloud();
                switch (RealtimeReconMode)
                {
                    case RTReconMode.RTRECONMODE_FAR:
                        DrawBPPointCloudToSurface();
                        break;
                    case RTReconMode.RTRECONMODE_NEAR:
                        DrawBPPointCloudToRealTimePointCloudRGB();
                        break;
                }
                
                //sw.Stop();
                //Debug.WriteLine("BP Draw Image tooks " + sw.ElapsedMilliseconds + " ms.");
            }
            //DrawBPPointCloudToRealTimePointCloudRGB();
        }

        private Vector3Collection RealtimeVector3s;
        private List<float[]> RealtimeUVs;
        private Color4Collection RealtimeCC;
        private PointGeometry3D realtimeReconPointCloud;
        public PointGeometry3D RealtimeReconPointCloud
        {
            get { return realtimeReconPointCloud; }
            set { realtimeReconPointCloud = value; OnPropertyChanged(nameof(RealtimeReconPointCloud)); }
        }
        private void DrawBPPointCloudToRealTimePointCloud()
        {
            List<LMData> tempListModeData = new List<LMData>();
            if (RealtimeVector3s == null || RealtimeVector3s.Count() == 0)
            {
                return;
            }
            if (LACC_Control_Static.ListedLMData.Count == 0)
            {
                return;
            }
            try
            {
                tempListModeData = (from LM in LACC_Control_Static.ListedLMData
                                    where LM != null && LM.MeasurementTime > DateTime.Now - TimeSpan.FromSeconds(MLPETime)
                                    select LM).ToList();
            }
            catch (NullReferenceException e)
            {
                Debug.WriteLine(e.ToString());
                return;
            }
            if (tempListModeData.Count == 0)
            {
                return;
            }
            //Trace.WriteLine("LM Data count " + tempListModeData.Count());

            var (v3, c4) = ImageRecon.BPtoPointCloud(RealtimeVector3s, tempListModeData, false ,1, 0.8);

           // RealtimeReconPointCloud = new PointGeometry3D() { Positions = v3, Colors = c4 };
        }

        private BitmapImage reconBitmapImage;
        public BitmapImage ReconBitmapImage
        {
            get { return reconBitmapImage; }
            set
            {
                reconBitmapImage = value;
                OnPropertyChanged(nameof(ReconBitmapImage));
            }
        }
        private eReconType selectReconType = eReconType.COMPTON;
        public eReconType SelectReconType
        {
            get
            {
                return selectReconType;
            }
            set
            {
                selectReconType = value;
                OnPropertyChanged(nameof(SelectReconType));
            }
        }

        private ObservableCollection<eReconType> selectReconTypes = new ObservableCollection<eReconType>(new eReconType[] { eReconType.COMPTON, eReconType.CODED, eReconType.HYBRID });
        public ObservableCollection<eReconType> SelectReconTypes
        {
            get
            {
                return selectReconTypes;
            }
            set
            {
                selectReconTypes = value;
                OnPropertyChanged(nameof(SelectReconTypes));
            }
        }
        private Vector3Collection SurfaceImageVector3;
        private List<float[]> SurfaceImageUVs;
        private void DrawBPPointCloudToRealTimePointCloudRGB()
        {
            List<double[]> color = new List<double[]>();
            List<float[]> uvs = new List<float[]>();
            try
            {


                LahgiWrapper_Static.GetRealTimeReconImage((double)MLPETime, ref color, ref uvs, selectReconType);

            }
            catch (NullReferenceException e)
            {
                Debug.WriteLine(e.ToString());
                return;
            }

            Bitmap bitmapOut = new Bitmap(ReconRGBPixelWidth, ReconRGBPixelHeight);

            for (int i = 0; i < uvs.Count; i++)
            {

                System.Drawing.Color bitMapColor = System.Drawing.Color.FromArgb(Convert.ToInt32(color[i][3] * 255),
                    Convert.ToInt32(color[i][0] * 255),
                    Convert.ToInt32(color[i][1] * 255),
                    Convert.ToInt32(color[i][2] * 255));
             
                int u = (int)(uvs[i][0] * ReconRGBPixelWidth);
                int v = (int)(uvs[i][1] * ReconRGBPixelHeight);
                if (u == ReconRGBPixelWidth)
                {
                    --u;
                }
                if (v == ReconRGBPixelHeight)
                {
                    --v;
                }

                bitmapOut.SetPixel(u, v, bitMapColor);

            }



            ReconBitmapImage = Bitmap2BitmapImage(bitmapOut);
            //RealtimeReconPointCloud = new PointGeometry3D() { Positions = v3, Colors = c4 };
        }

        private void DrawBPPointCloudToSurface()
        {
            List<LMData> tempListModeData = new List<LMData>();         
            if (LACC_Control_Static.ListedLMData.Count == 0)
            {
                return;
            }
            try
            {
                tempListModeData = (from LM in LACC_Control_Static.ListedLMData
                                    where LM != null && LM.MeasurementTime > DateTime.Now - TimeSpan.FromSeconds(MLPETime)
                                    select LM).ToList();
            }
            catch (NullReferenceException e)
            {
                Debug.WriteLine(e.ToString());
                return;
            }
            if (tempListModeData.Count == 0)
            {
                return;
            }
            //Trace.WriteLine("LM Data count " + tempListModeData.Count());
            var tempVector3s = SurfaceImageVector3;
            var tempUVs = SurfaceImageUVs;
            var (v3, c4, bitmap) = ImageRecon.BPtoPointCloudBitmap(tempVector3s, tempUVs, tempListModeData, ReconRGBPixelHeight, ReconRGBPixelWidth, false, 5, 0.8);

            ReconBitmapImage = Bitmap2BitmapImage(bitmap);
            //RealtimeReconPointCloud = new PointGeometry3D() { Positions = v3, Colors = c4 };
        }

        private Vector3Collection SLAMVector3s = new Vector3Collection();
        private PointGeometry3D slamReconPointCloud;
        public PointGeometry3D SLAMReconPointCloud
        {
            get { return slamReconPointCloud; }
            set { 
                slamReconPointCloud = value; 
                OnPropertyChanged(nameof(SLAMReconPointCloud)); }
        }

        private Task SLAMReconTaskAsync;
        private void SLAMRecon()
        {
            while (IsRealTimeImageReconOn && IsSLAMOn)
            {
                //Stopwatch sw = new Stopwatch();
                //sw.Start();
                DrawBPPointCloudToSLAMPointCloud();
                Thread.Sleep(1000);
                //sw.Stop();
                //Debug.WriteLine("BP Draw Image tooks " + sw.ElapsedMilliseconds + " ms.");
            }
            DrawBPPointCloudToSLAMPointCloud();
        }
        long elapsedTime = 0;
        void DrawBPPointCloudToSLAMPointCloud()
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();
            VMStatus = $"Reconing......SLAM! {elapsedTime} ms";


            Thread.Sleep(500);
            var vc = new Vector3Collection();
            var cc = new Color4Collection();
            var tempposeVect = new List<double[]>();
            var tempColorVect = new List<double[]>();


            RealsenseControl.GetReconSLAMPointCloud(ref tempposeVect, ref tempColorVect);
            for (int i = 0; i < tempposeVect.Count; i++)
            {
                vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][1]), 0.5f));
                //cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.5f));
                //id.Add(i);
            }


            if (tempposeVect == null || tempposeVect.Count() == 0)
            {
                return;
            }
            List<LMData> tempListModeData = new List<LMData>();
            if (tempposeVect == null || tempposeVect.Count() == 0)
            {
                return;
            }
            if (LACC_Control_Static == null || LACC_Control_Static.ListedLMData.Count == 0)
            {
                return;
            }
            
            tempListModeData = (from LM in LACC_Control_Static.ListedLMData
                                where LM != null && LM.MeasurementTime > DateTime.Now - TimeSpan.FromSeconds(6000)
                                select LM).ToList();

            var (v3, c4) = ImageRecon.BPtoPointCloud2Pi(vc, tempListModeData, 5, 0.8);            
            SLAMReconPointCloud = new PointGeometry3D() { Positions = v3, Colors = c4 };
            SLAMPointCloudCount = vc.Count();
            sw.Stop();
            elapsedTime = sw.ElapsedMilliseconds;
            VMStatus = "Reconing......SLAM Done!";

        }
        #endregion


    }


}
