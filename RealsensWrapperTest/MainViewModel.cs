using GalaSoft.MvvmLight.Command;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton.LACC;
using Microsoft.Win32;
using SharpDX;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;

namespace RealsensWrapperTest
{
    public class MainViewModel: INotifyPropertyChanged
    {
        private RealsenseControlWrapper RealsenseControl;

        public MainViewModel()
        {
            RealsenseControl = new RealsenseControlWrapper();
            InitiateRealsense();
        }

        private Vector3D T265ToLACCOffset = new Vector3D(0, 0, 0);

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
            if(!IsRealsensePipelineOn)
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
            set { 
                realtimeRGB = value; 
                OnPropertyChanged(nameof(RealtimeRGB)); 
            }
        }

        private Task UpdateRGBPoseTask;
        private void UpdateRGBPose()
        {          
            Thread.Sleep(0);
            while(IsRealsensePipelineOn)
            {
                var marix3DElement = RealsenseControl.GetPoseFrame();
                if (marix3DElement != null)
                {
                    IsT265TrackingConfidence3 = true;
                    CurrentPose = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                            marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7],
                                            marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11],
                                            marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);
                }
                else
                {
                    IsT265TrackingConfidence3 = false;
                    CurrentPose = new Matrix3D(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
                }

                Bitmap tempBitmap = new Bitmap(1,1);
                RealsenseControl.GetRealTimeRGB(ref tempBitmap);
                //tempBitmap.Save("E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20201203 Comtpon GUI\\Compton GUI Main\\HUREL Compton\\RealsensWrapperTest\\bin\\Debug\\net5.0-windows\\test.png");
                // Bitmap 담을 메모리스트림 
                if (tempBitmap == null || tempBitmap.Width == 1)
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

        private Matrix3D currentPose;
        private Matrix3D CurrentPose
        {
            get { return currentPose; }
            set
            {
                currentPose = value;
                PoseX = value.OffsetX;
                PoseY = value.OffsetY;
                PoseZ = value.OffsetZ;
            }
        }

        private double poseX;
        public double PoseX
        {
            get { return poseX; }
            set { poseX = value; OnPropertyChanged(nameof(PoseX)); }
        }

        private double poseY;
        public double PoseY
        {
            get { return poseY; }
            set { poseY = value; OnPropertyChanged(nameof(PoseY)); }
        }

        private double poseZ;
        public double PoseZ
        {
            get { return poseZ; }
            set { poseZ = value; OnPropertyChanged(nameof(PoseZ)); }
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
                    RealsenseControl.GetRealTimePointCloud(ref poseVect, ref colorVect, T265ToLACCOffset.X, T265ToLACCOffset.Y,T265ToLACCOffset.Z);

                    for (int i = 0; i < poseVect.Count; i++)
                    {

                        vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                        cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.8f));
                        //id.Add(i);
                    }


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
            Thread.Sleep(2000);
            SLAMPointCloud = new PointGeometry3D();
            SLAMPoseInfo = new LineGeometry3D();
            UpdateSLAMPointCloudTask = Task.Run(() => UpdateSLAMPointCloud());
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
        }


        private Task UpdateSLAMPointCloudTask;
        public void UpdateSLAMPointCloud()
        {

            int Error = 0;
            var pose = new List<HelixToolkit.Wpf.SharpDX.Geometry3D.Line>();
            var line = new LineBuilder();
            Vector3 previousPose = new Vector3(Convert.ToSingle(poseX), Convert.ToSingle(poseY), Convert.ToSingle(poseZ));

            while (IsSLAMOn)
            {
                var vc = new Vector3Collection();
                var id = new IntCollection();
                var cc = new Color4Collection();

                var poseVect = new List<double[]>();
                var colorVect = new List<double[]>();
                Vector3 currentPose = new Vector3(Convert.ToSingle(poseX), Convert.ToSingle(poseY), Convert.ToSingle(poseZ));

                line.AddLine(previousPose, currentPose);

                SLAMPoseInfo = line.ToLineGeometry3D();
                previousPose = currentPose;

                try
                {
                    RealsenseControl.GetSLAMPointCloud(ref poseVect, ref colorVect, T265ToLACCOffset.X, T265ToLACCOffset.Y, T265ToLACCOffset.Z);

                    for (int i = 0; i < poseVect.Count; i++)
                    {

                        vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                        cc.Add(new Color4(0.1f, 0.1f, 0.1f, 0.5f));
                        //cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.5f));
                        id.Add(i);
                    }
                    SLAMPointCloud = new PointGeometry3D() { Positions = vc, Indices = id, Colors = cc };
                }
                catch
                {
                    Error++;
                    Trace.WriteLine("Error Count is " + Error);
                }
                Thread.Sleep(50);
            }
        }


        #endregion

        private string vmStatus;
        public string VMStatus
        {
            get { return vmStatus; }
            set { vmStatus = value; OnPropertyChanged(nameof(VMStatus)); }
        }

        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    }
}
