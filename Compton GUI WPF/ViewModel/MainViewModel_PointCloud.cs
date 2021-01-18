using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using SLAMwrapper;
using System.Diagnostics;
using GalaSoft.MvvmLight.Command;
using System.Windows.Input;
using HUREL.Compton;
using System.Windows.Media.Media3D;

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel
    {
        private int mlpeTime;
        public int MLPETime
        {
            get { return mlpeTime; }
            set { mlpeTime = value; OnPropertyChanged(nameof(MLPETime)); }
        }
        public void AddMonoRealTimeListModeDataToComptonBP(Vector3Collection vector3s, Color4Collection color4s)
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();
            if (recordTimeSpan.Seconds < mlpeTime + 5)
            {
                isBPNeedUpdated = true;
                return;
            }

            var timeSortedLMData = (from lmData in LACC_Control_Static.ListedLMData
                                   where DateTime.Now.Subtract(lmData.MeasurementTime).TotalSeconds < mlpeTime
                                   select lmData).ToList();
            Debug.WriteLine("LinQ Time: " + sw.ElapsedMilliseconds + " ms");
            var bpData =  Backprojection.BPtoPointCloud(vector3s, color4s, timeSortedLMData);
            bpVectors = bpData.Item1.ToList();
            bpColor4s = bpData.Item2.ToList();
            Debug.WriteLine("Adding Elasped Time: " + sw.ElapsedMilliseconds + " ms");
            isBPNeedUpdated = true;
        }

        private List<Vector3> bpVectors = new List<Vector3>();
        private List<Color4> bpColor4s = new List<Color4>();

        private static Realsense_Control rsControl = new Realsense_Control();

        private Task RTPointCloudTask;
        private bool IsRealTimePointCloudOn = true;
        public void GetRealTimePointCloud()
        {
            rsControl.InitiateRealsense();
            int Error = 0;
            Thread.Sleep(3000);
            while (IsRealTimePointCloudOn)
            {
                var vc = new Vector3Collection();
                var id = new IntCollection();
                var cc = new Color4Collection();

                var poseVect = new List<double[]>();
                var colorVect = new List<double[]>();
                //Debug.WriteLine("Start to get Reatime Data");
                try
                {
                    rsControl.GetRealTimePointCloud(ref poseVect, ref colorVect, isMLPEOn);

                    for (int i = 0; i < poseVect.Count; i++)
                    {

                        vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                        cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.8f));
                        //id.Add(i);
                    }


                    if (isMLPEOn && isSessionStart && !isBPNeedUpdated)
                    {
                        var bpvc = new Vector3Collection();
                        var bpcc = new Color4Collection();
                        bpvc.AddRange(bpVectors);
                        bpcc.AddRange(bpColor4s);
                        BPPointCloud = new PointGeometry3D() { Positions = bpvc, Colors = bpcc };
                        RTPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };
                    }
                    else if(isMLPEOn && isSessionStart && isBPNeedUpdated)
                    {
                        isBPNeedUpdated = false;
                        Task.Run(() => AddMonoRealTimeListModeDataToComptonBP(vc, cc));

                        RTPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };
                        
                    }
                    else 
                    {
                        RTPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };
                    }

                    var marix3DElement = rsControl.GetPoseFrame();
                   if (marix3DElement != null)
                    {
                        IsTrackingConfidence3 = true;
                        CurrentPos = new Matrix3D(marix3DElement[0], marix3DElement[1], marix3DElement[2], marix3DElement[3],
                                                marix3DElement[4], marix3DElement[5], marix3DElement[6], marix3DElement[7], 
                                                marix3DElement[8], marix3DElement[9], marix3DElement[10], marix3DElement[11], 
                                                marix3DElement[12], marix3DElement[13], marix3DElement[14], marix3DElement[15]);
                    }
                }
                catch
                {
                    Error++;
                    Trace.WriteLine("Error Count is " + Error);
                }
                if(isMLPEOn&& isSessionStart)
                { }
                Thread.Sleep(10);


            }

        }

        private bool isBPNeedUpdated= true;

        public void AddPoints()
        {
            var vc = new Vector3Collection();
            var id = new IntCollection();
            var cc = new Color4Collection();

            vc.Add(new Vector3(0, 0, 0));
            cc.Add(new Color4(1, 0, 0, 1));
            id.Add(0);

            vc.Add(new Vector3(2, 0, 0));
            cc.Add(new Color4(1, 0, 0, 1));
            id.Add(1);

            vc.Add(new Vector3(0, 2, 0));
            cc.Add(new Color4(1, 0, 0, 1));
            id.Add(2);

            vc.Add(new Vector3(0, 0, 2));
            cc.Add(new Color4(0, 1, 0, 1));
            id.Add(3);

            vc.Add(new Vector3(2, 2, 0));
            cc.Add(new Color4(0, 1, 0, 1));
            id.Add(4);

            vc.Add(new Vector3(0, 2, 2));
            cc.Add(new Color4(0, 0, 1, 1));
            id.Add(5);

            vc.Add(new Vector3(2, 0, 2));
            cc.Add(new Color4(0, 0, 1, 1));
            id.Add(6);

            RTPointCloud = new PointGeometry3D() { Positions = vc, Indices = id, Colors = cc };


        }

        private PointGeometry3D rtPointCloud;
        public PointGeometry3D RTPointCloud
        {
            get { return rtPointCloud; }
            set { rtPointCloud = value; OnPropertyChanged(nameof(RTPointCloud)); }
        }

        private PointGeometry3D bpPointCloud;
        public PointGeometry3D BPPointCloud
        {
            get { return bpPointCloud; }
            set { bpPointCloud = value; OnPropertyChanged(nameof(BPPointCloud)); }
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
            rsControl.StartSLAM();
            Thread.Sleep(2000);
            Task.Run(() => GetSLAMPointCloud()) ;
        }
        private RelayCommand stopSLAMCommand;
        public ICommand StopSLAMCommand
        {
            get { return (this.stopSLAMCommand) ?? (this.stopSLAMCommand = new RelayCommand(StopSLAM)); }
        }

        private void StopSLAM()
        {
            rsControl.StopSLAM();

            var bpvc = new Vector3Collection();
            var bpcc = new Color4Collection();
           
            bpvc.AddRange(SLAMPointCloud.Positions);
            bpcc.AddRange(SLAMPointCloud.Colors);


            Task.Run(() => AddMonoSlamListModeDataToComptonBP(bpvc, bpcc));
            IsSLAMOn = false;
        }

        public void AddMonoSlamListModeDataToComptonBP(Vector3Collection vector3s, Color4Collection color4s)
        {
            VMStatus = "Running BP To SLAM";
            Stopwatch sw = new Stopwatch();
            sw.Start();

            var timeSortedLMData = LACC_Control_Static.ListedLMData;
                                    
            Debug.WriteLine("LinQ Time: " + sw.ElapsedMilliseconds + " ms");
            var bpData = Backprojection.BPtoPointCloudSLAM(vector3s, color4s, timeSortedLMData);
            bpVectors = bpData.Item1.ToList();
            bpColor4s = bpData.Item2.ToList();

            var bpvc = new Vector3Collection();
            var bpcc = new Color4Collection();

            bpvc.AddRange(bpVectors);
            bpcc.AddRange(bpColor4s);

            Debug.WriteLine("Adding Elasped Time: " + sw.ElapsedMilliseconds + " ms");
            VMStatus = "BP To SLAM is Done!";
            SLAMBPPointCloud = new PointGeometry3D() { Positions = bpvc, Colors = bpcc };
        }

        public void GetSLAMPointCloud()
        {

            int Error = 0;

            while (IsSLAMOn)
            {
                var vc = new Vector3Collection();
                var id = new IntCollection();
                var cc = new Color4Collection();

                var poseVect = new List<double[]>();
                var colorVect = new List<double[]>();
                //Debug.WriteLine("Start to get Reatime Data");
                try
                {
                    rsControl.GetSLAMPointCloud(ref poseVect, ref colorVect);

                    for (int i = 0; i < poseVect.Count; i++)
                    {

                        vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                        cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 0.5f));
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

        private PointGeometry3D slamPointCloud;
        public PointGeometry3D SLAMPointCloud
        {
            get { return slamPointCloud; }
            set { slamPointCloud = value; OnPropertyChanged(nameof(SLAMPointCloud)); }
        }

        private PointGeometry3D slamBPPointCloud;
        public PointGeometry3D SLAMBPPointCloud
        {
            get { return slamBPPointCloud; }
            set { slamBPPointCloud = value; OnPropertyChanged(nameof(SLAMBPPointCloud)); }
        }

    }
}
