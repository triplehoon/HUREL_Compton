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

namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel
    {

        private static Realsense_Control rsControl = new Realsense_Control();
        public void GetRealTimePointCloud()
        {
            rsControl.InitiateRealsense();
            int Error = 0;
            Thread.Sleep(5000);
            while (true)
            {
                var vc = new Vector3Collection();
                var id = new IntCollection();
                var cc = new Color4Collection();

                var poseVect = new List<double[]>();
                var colorVect = new List<double[]>();
                //Debug.WriteLine("Start to get Reatime Data");
                try
                {

                    rsControl.GetRealTimePointCloud(ref poseVect, ref colorVect);
                    //Debug.WriteLine("PoseVect Count is " + poseVect.Count);
                    for (int i = 0; i < poseVect.Count; i++)
                    {

                        vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));
                        cc.Add(new Color4(Convert.ToSingle(colorVect[i][0]), Convert.ToSingle(colorVect[i][1]), Convert.ToSingle(colorVect[i][2]), 1));
                        id.Add(i);
                    }
                    RTPointCloud = new PointGeometry3D() { Positions = vc, Indices = id, Colors = cc };
                }
                catch
                {
                    Error++;
                    Trace.WriteLine("Error Count is " + Error);
                }

                Thread.Sleep(30);


            }

        }

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
    }
}
