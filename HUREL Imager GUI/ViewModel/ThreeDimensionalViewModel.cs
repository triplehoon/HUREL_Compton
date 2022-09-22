using AsyncAwaitBestPractices.MVVM;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;

namespace HUREL_Imager_GUI.ViewModel
{
    public class ThreeDimensionalViewModel : ViewModelBase
    {

        public ThreeDimensionalViewModel()
        {           
            LahgiApi.StatusUpdate += updateView;

            var vc = new Vector3Collection();
            var cc = new Color4Collection();

            for (int i = 0; i < 10; ++i)
            {
                var pointx = new Vector3(0.1f * i, 0, 0);
                var pointy = new Vector3(0, 0.1f * i, 0);
                var pointz = new Vector3(0, 0, 0.1f * i);
                vc.Add(new Vector3(Convert.ToSingle(pointx.X), Convert.ToSingle(pointx.Y), Convert.ToSingle(pointx.Z)));
                cc.Add(new Color4(1f, 0f, 0f, 1f));
                vc.Add(new Vector3(Convert.ToSingle(pointy.X), Convert.ToSingle(pointy.Y), Convert.ToSingle(pointy.Z)));
                cc.Add(new Color4(0f, 1f, 0f, 1f));
                vc.Add(new Vector3(Convert.ToSingle(pointz.X), Convert.ToSingle(pointz.Y), Convert.ToSingle(pointz.Z)));
                cc.Add(new Color4(0f, 0f, 1f, 1f));
            }     

            SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

            Text3d = new BillboardText3D();
            TextInfoExt tinfo = new TextInfoExt();
            tinfo.Text = ("Cs-137");
            tinfo.Origin = new Vector3(-1, 0.2f, 2.5f);
            tinfo.Scale = 1f;
            Text3d.TextInfo.Add(tinfo);
        }





        private BillboardText3D text3d = new BillboardText3D();
        public BillboardText3D Text3d
        {
            get { return text3d; }
            set
            {
                text3d = value;
                OnPropertyChanged(nameof(Text3d));
            }
        }



        private void updateView(object? obj, EventArgs args)
        {
            Trace.WriteLine("Update");

            if (args is LahgiApiEnvetArgs)
            { 
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)args;

                if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                {
                    UpdateLoadedSlamPointCloud();
                }             
                else if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamPoints)
                {
                    UpdateRealtimeSlamPointCloud();
                }
                else if(lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamRadImage)
                {
                    UpateRealtimeReconSlamPointCloud();
                }                
            }
        }

        private static Mutex updateLoadDataMutex = new Mutex();
        private void UpdateLoadedSlamPointCloud()
        {            
            Task.Run(() =>
            {
                updateLoadDataMutex.WaitOne();
                var vc = new Vector3Collection();
                var cc = new Color4Collection();
                var tempposeVect = new List<double[]>();
                var tempColorVect = new List<double[]>();
                List<float[]> uvs = new List<float[]>();
                LahgiApi.GetLoadedPointCloud(ref tempposeVect, ref tempColorVect);
                for (int i = 0; i < tempposeVect.Count; i++)
                {
                    vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                    cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                }

                SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

                var vc2 = new Vector3Collection();
                var cc2 = new Color4Collection();
                LahgiApi.GetReconSLAMPointCloud(0, eReconManaged.COMPTON, ref tempposeVect, ref tempColorVect, 0.01, true);
                for (int i = 0; i < tempposeVect.Count; i++)
                {
                    vc2.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                    cc2.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
                }
                
                SLAMReconPointCloud = new PointGeometry3D() { Positions = vc2, Colors = cc2 };
                
                updateLoadDataMutex.ReleaseMutex();
            });
        }

        private void UpdateRealtimeSlamPointCloud()
        {
            if(!updateLoadDataMutex.WaitOne(100))
            {
                return;
            };
            var vc = new Vector3Collection();
            var cc = new Color4Collection();
            var tempposeVect = new List<double[]>();
            var tempColorVect = new List<double[]>();
            List<float[]> uvs = new List<float[]>();
            LahgiApi.GetSLAMPointCloud(ref tempposeVect, ref tempColorVect);
            for (int i = 0; i < tempposeVect.Count; i++)
            {
                vc.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                cc.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
            }

            SLAMPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };

            UpdateRealtimeSlamPoseGraph();


            updateLoadDataMutex.ReleaseMutex();


        }
        private void UpdateRealtimeSlamPoseGraph()
        {
            var line = new LineBuilder();
            List<Matrix3D> poses = new List<Matrix3D>();
            if (LahgiApi.GetOptimizedPoses(ref poses))
            {
                
                line.AddLine(new Vector3(0, 0, 0), new Vector3((float)poses[0].OffsetX, (float)poses[0].OffsetY, (float)poses[0].OffsetZ));
                for (int i = 1; i < poses.Count; i++)
                {
                    line.AddLine(
                        new Vector3((float)poses[i - 1].OffsetX, (float)poses[i - 1].OffsetY, (float)poses[i - 1].OffsetZ),
                        new Vector3((float)poses[i].OffsetX, (float)poses[i].OffsetY, (float)poses[i].OffsetZ));
                }
                SLAMPoseInfo = line.ToLineGeometry3D();
            }

        }
        private static Mutex updateReconSlamPointMutex = new Mutex();
        private void UpateRealtimeReconSlamPointCloud()
        {
            updateReconSlamPointMutex.WaitOne(100);
            var vc2 = new Vector3Collection();
            var cc2 = new Color4Collection();
            var tempposeVect = new List<double[]>();
            var tempColorVect = new List<double[]>();
            LahgiApi.GetReconSLAMPointCloud(0, eReconManaged.COMPTON, ref tempposeVect, ref tempColorVect, 0.01, false);
            for (int i = 0; i < tempposeVect.Count; i++)
            {
                vc2.Add(new Vector3(Convert.ToSingle(tempposeVect[i][0]), Convert.ToSingle(tempposeVect[i][1]), Convert.ToSingle(tempposeVect[i][2])));
                cc2.Add(new Color4(Convert.ToSingle(tempColorVect[i][0]), Convert.ToSingle(tempColorVect[i][1]), Convert.ToSingle(tempColorVect[i][2]), Convert.ToSingle(tempColorVect[i][3])));
            }

            SLAMReconPointCloud = new PointGeometry3D() { Positions = vc2, Colors = cc2 };
            updateReconSlamPointMutex.ReleaseMutex();
        }



        private PointGeometry3D slamPointCloud = new PointGeometry3D();
        public PointGeometry3D SLAMPointCloud
        {
            get { return slamPointCloud; }
            set
            {
                slamPointCloud = value;
                OnPropertyChanged(nameof(SLAMPointCloud));
            }
        }
        
        private PointGeometry3D slamReconPointCloud = new PointGeometry3D();
        public PointGeometry3D SLAMReconPointCloud
        {
            get { return slamReconPointCloud; }
            set
            {
                slamReconPointCloud = value;
                OnPropertyChanged(nameof(SLAMReconPointCloud));
            }
        }
        private LineGeometry3D slamPoseInfo;
        public LineGeometry3D SLAMPoseInfo
        {
            get { return slamPoseInfo; }
            set { slamPoseInfo = value; OnPropertyChanged(nameof(SLAMPoseInfo)); }
        }


        private AsyncCommand? startSlamCommand = null;
        public ICommand StartSlamCommand
        {
            get { return startSlamCommand ?? (startSlamCommand = new AsyncCommand(StartSlam)); }
        }
        private async Task StartSlam()
        {
            await Task.Run(() =>
            {
                LahgiApi.StartSlam();
            });
        }


        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= updateView;
        }
    }
}
