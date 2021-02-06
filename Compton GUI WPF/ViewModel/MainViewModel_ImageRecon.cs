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

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel
    {
        private RealsenseControlWrapper RealsenseControl = new RealsenseControlWrapper();

        private Vector3D T265ToLACCOffset = new Vector3D(0, 0.24, +0.03);
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

        private RelayCommand initiateRealsenseCommand;
        public ICommand InitiateRealsenseCommand
        {
            get { return (this.initiateRealsenseCommand) ?? (this.initiateRealsenseCommand = new RelayCommand(InitiateRealsense)); }
        }
        private void InitiateRealsense()
        {
            RealsenseState = "Initiating Realsense";
            Task.Run(() =>
            {
                string s = "";
                IsRealsenseOn = RealsenseControl.InitiateRealsense(ref s);
                RealsenseState = s;
                StartRealsensePipeline();
            });

            
        }

        #region Realsense Pipeline Setup

        private bool isRealsensePipelineOn;
        public bool IsRealsensePipelineOn
        {
            get { return isRealsensePipelineOn; }
            set { isRealsensePipelineOn = value; OnPropertyChanged(nameof(IsRealsensePipelineOn)); }
        }
        private RelayCommand startRealsensePipelineCommand;
        public ICommand StartRealsensePipelineCommand
        {
            get { return (this.startRealsensePipelineCommand) ?? (this.startRealsensePipelineCommand = new RelayCommand(StartRealsensePipeline)); }
        }

        private void StartRealsensePipeline()
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
        }

        private RelayCommand resetRealsensePipelineCommand;
        public ICommand ResetRealsensePipelineCommand
        {
            get { return (this.resetRealsensePipelineCommand) ?? (this.resetRealsensePipelineCommand = new RelayCommand(ResetRealsensePipeline)); }
        }

        private void ResetRealsensePipeline()
        {
            Task.Run(() =>
            {
                string temp = "";
                RealsenseControl.ResetPipeline(ref temp);
                RealsenseState = temp;
            });

        }

        private RelayCommand stopRealsensePipelineCommand;
        public ICommand StopRealsensePipelineCommand
        {
            get { return (this.stopRealsensePipelineCommand) ?? (this.stopRealsensePipelineCommand = new RelayCommand(StopRealsensePipeline)); }
        }
        private void StopRealsensePipeline()
        {
            if (!IsRealsensePipelineOn)
            {
                RealsenseState = "Pipeline is not on";
                return;
            }
            RealsenseControl.StopRealsensePipeline();
            IsRealsensePipelineOn = false;
            UpdateRGBPoseTask.Wait();
            UpdateRealTimePointCloudTask.Wait();
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
                if (tempBitmap.Width == 1)
                {
                    continue;
                }

                MemoryStream ms = new MemoryStream();
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

        private Matrix3D currentSystemPose;
        private Matrix3D CurrentSystemTranformation
        {
            get { return currentSystemPose; }
            set
            {
                currentSystemPose = value;
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
        
        private Task UpdateRealTimePointCloudTask;
        public void UpdateRealTimePointCloud()
        {
            int error = 0;
            while (IsRealsensePipelineOn)
            {
                var vc = new Vector3Collection();
                var id = new IntCollection();
                var cc = new Color4Collection();

                var poseVect = new List<double[]>();
                var colorVect = new List<double[]>();
                //Debug.WriteLine("Start to get Reatime Data");
                try
                {
                    RealsenseControl.GetRealTimePointCloud(ref poseVect, ref colorVect);
                    for (int i = 0; i < poseVect.Count; i++)
                    {

                        vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                        cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.8f));
                        //id.Add(i);
                    }

                    RealtimeVector3s = vc;
                    RTPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

                }
                catch
                {
                    error++;
                    Trace.WriteLine("Error Count is " + error);
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

        private bool IsSLAMOn = false;

        private RelayCommand startSLAMCommand;
        public ICommand StartSLAMCommand
        {
            get { return (this.startSLAMCommand) ?? (this.startSLAMCommand = new RelayCommand(StartSLAM)); }
        }
        private void StartSLAM()
        {
            IsSLAMOn = true;
            string temp = "";
            RealsenseControl.StartSLAM(ref temp);
            RealsenseState = temp;
            SLAMPointCloud = new PointGeometry3D();
            SLAMPoseInfo = new LineGeometry3D();
            UpdateSLAMPointCloudTask = Task.Run(() => UpdateSLAMPointCloud());
            //SLAMReconTask = Task.Run(() => SLAMRecon());
        }


        private RelayCommand stopSLAMCommand;
        public ICommand StopSLAMCommand
        {
            get { return (this.stopSLAMCommand) ?? (this.stopSLAMCommand = new RelayCommand(StopSLAM)); }
        }
        private void StopSLAM()
        {
            RealsenseControl.StopSLAM();
            IsSLAMOn = false;
            UpdateSLAMPointCloudTask.Wait();
            //SLAMReconTask.Wait();
        }

        private Task UpdateSLAMPointCloudTask;
        public void UpdateSLAMPointCloud()
        {

            int Error = 0;
            var pose = new List<HelixToolkit.Wpf.SharpDX.Geometry3D.Line>();
            var line = new LineBuilder();
            Vector3 previousPose = new Vector3(Convert.ToSingle(systemPoseX), Convert.ToSingle(systemPoseY), Convert.ToSingle(systemPoseZ));
           

            var poseVect = new List<double[]>();
            var colorVect = new List<double[]>();

            while (IsSLAMOn)
            {
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
                    cc.Add(new Color4(0.1f, 0.1f, 0.1f, 0.5f));
                    //cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.5f));
                    //id.Add(i);
                }
                SLAMVector3s = vc;
                SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };
                SLAMPointCloudCount = vc.Count();


                //Thread.Sleep(500);                
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

        private Task RealTimeImageReconTask;
        private void RealTimeImageRecon()
        {
            while (IsRealTimeImageReconOn)
            {
                //Stopwatch sw = new Stopwatch();
                //sw.Start();
                DrawBPPointCloudToRealTimePointCloud();
                //sw.Stop();
                //Debug.WriteLine("BP Draw Image tooks " + sw.ElapsedMilliseconds + " ms.");
            }
            DrawBPPointCloudToRealTimePointCloud();
        }

        private Vector3Collection RealtimeVector3s;
        private PointGeometry3D realtimeReconPointCloud;
        public PointGeometry3D RealtimeReconPointCloud
        {
            get { return realtimeReconPointCloud; }
            set { realtimeReconPointCloud = value; OnPropertyChanged(nameof(RealtimeReconPointCloud)); }
        }
        void DrawBPPointCloudToRealTimePointCloud()
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

            var (v3, c4) = ImageRecon.BPtoPointCloud(RealtimeVector3s, tempListModeData, false ,5, 0.8);

            RealtimeReconPointCloud = new PointGeometry3D() { Positions = v3, Colors = c4 };
        }

        private Vector3Collection SLAMVector3s;
        private PointGeometry3D slamReconPointCloud;
        public PointGeometry3D SLAMReconPointCloud
        {
            get { return slamReconPointCloud; }
            set { slamReconPointCloud = value; OnPropertyChanged(nameof(SLAMReconPointCloud)); }
        }

        private Task SLAMReconTask;
        private void SLAMRecon()
        {
            while (IsRealTimeImageReconOn && IsSLAMOn)
            {
                //Stopwatch sw = new Stopwatch();
                //sw.Start();
                DrawBPPointCloudToSLAMPointCloud();
                //sw.Stop();
                //Debug.WriteLine("BP Draw Image tooks " + sw.ElapsedMilliseconds + " ms.");
            }
            DrawBPPointCloudToSLAMPointCloud();
        }
        void DrawBPPointCloudToSLAMPointCloud()
        {
            if (SLAMVector3s == null || SLAMVector3s.Count() == 0)
            {
                return;
            }
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
                                     where LM != null && LM.MeasurementTime > DateTime.Now - TimeSpan.FromSeconds(3000)
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
         
                                             

            var (v3, c4) = ImageRecon.BPtoPointCloud(SLAMVector3s, tempListModeData, true, 5, 0.6);
            Trace.WriteLine(v3.Count());
            SLAMReconPointCloud = new PointGeometry3D() { Positions = v3, Colors = c4 };
                
        }
        #endregion


    }


}
