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
using System.Windows.Media;

namespace Compton_GUI_WPF.ViewModel
{

    public enum eProcessingType
    {
        SimpleBP,
        MLEM
    }

    public partial class MainViewModel
    {
        #region Pointcloud
        private PointGeometry3D rtPointCloud;
        public PointGeometry3D RTPointCloud
        {
            get { return rtPointCloud; }
            set { rtPointCloud = value; OnPropertyChanged(nameof(RTPointCloud)); }
        }

        private LineGeometry3D xAxisLine;
        public LineGeometry3D XAxisLine
        {
            get { return xAxisLine; }
            set { xAxisLine = value; OnPropertyChanged(nameof(XAxisLine)); }
        }

        private LineGeometry3D yAxisLine;
        public LineGeometry3D YAxisLine
        {
            get { return yAxisLine; }
            set { yAxisLine = value; OnPropertyChanged(nameof(YAxisLine)); }
        }

        private LineGeometry3D zAxisLine;
        public LineGeometry3D ZAxisLine
        {
            get { return zAxisLine; }
            set { zAxisLine = value; OnPropertyChanged(nameof(ZAxisLine)); }
        }


        private void InitiateAxis()
        {           
            var line = new LineBuilder();
            Vector3 originePose = new Vector3(0, 0, 0);
            Vector3 xPose = new Vector3(1, 0, 0);
            Vector3 yPose = new Vector3(0, 1, 0);
            Vector3 zPose = new Vector3(0, 0, 1);

            line.AddLine(originePose, xPose);
            XAxisLine = line.ToLineGeometry3D();

            line = new LineBuilder();
            line.AddLine(originePose, yPose);
            YAxisLine = line.ToLineGeometry3D();

            line = new LineBuilder();
            line.AddLine(originePose, zPose);
            ZAxisLine = line.ToLineGeometry3D();

        }
        #endregion

        #region ImageRecon
        private bool isRealTimeImageReconOn = true;
        public bool IsRealTimeImageReconOn
        {
            get { return isRealTimeImageReconOn; }
            set { isRealTimeImageReconOn = value; OnPropertyChanged(nameof(IsRealTimeImageReconOn)); }
        }

        public enum EReconProjection
        {
            XY,
            XZ,
            ZY
        }
        private EReconProjection reconProjection = EReconProjection.XY;
        public EReconProjection ReconProjection
        {
            get
            {
                return reconProjection;
            }
            set
            {
                reconProjection = value;
                OnPropertyChanged(nameof(ReconProjection));
                UpdatePointCloudProjection();
            }
        }

        private double projectionX = 0;
        public double ProjectionX
        {
            get
            {
                return projectionX;
            }
            set
            {
                projectionX = value;
                UpdatePointCloudProjection();
                OnPropertyChanged(nameof(ProjectionX));
            }
        }

        private double projectionY = 0;
        public double ProjectionY
        {
            get
            {
                return projectionY;
            }
            set
            {
                projectionY = value;
                UpdatePointCloudProjection();
                OnPropertyChanged(nameof(ProjectionY));
            }
        }

        private double projectionZ = 0;
        public double ProjectionZ
        {
            get
            {
                return projectionZ;
            }
            set
            {
                projectionZ = value;
                UpdatePointCloudProjection();
                OnPropertyChanged(nameof(ProjectionZ));
            }
        }


        private double projectionMinValue = 0;
        public double ProjectionMinValue
        {
            get
            {
                return projectionMinValue;
            }
            set
            {
                projectionMinValue = value;
                OnPropertyChanged(nameof(ProjectionMinValue));
                UpdatePointCloudProjection();
            }
        }


        private double projectionMaxValue = 100;
        public double ProjectionMaxValue
        {
            get
            {
                return projectionMaxValue;
            }
            set
            {
                projectionMaxValue = value;
                OnPropertyChanged(nameof(ProjectionMaxValue));
                UpdatePointCloudProjection();

            }
        }
        private double projectionTickValue = 0;
        public double ProjectionTickValue
        {
            get
            {
                return projectionTickValue;
            }
            set
            {
                projectionTickValue = value;
                OnPropertyChanged(nameof(ProjectionTickValue));
            }
        }

        private double trueReconMaxValue = 0;
        public double TrueReconMaxValue
        {
            get
            {
                return trueReconMaxValue;
            }
            set
            {
                trueReconMaxValue = value;
                OnPropertyChanged(nameof(TrueReconMaxValue));
                ProjectionTickValue = (trueReconMaxValue - 0) / 10;
            }
        }

        private eProcessingType selectReconType = eProcessingType.SimpleBP;
        public eProcessingType SelectReconType
        {
            get
            {
                return selectReconType;
            }
            set
            {
                selectReconType = value;               
                OnPropertyChanged(nameof(SelectReconType));
                UpdatePointCloudProjection();
            }
        }

        private ObservableCollection<eProcessingType> selectReconTypes = new ObservableCollection<eProcessingType>(new eProcessingType[] { eProcessingType.SimpleBP, eProcessingType.MLEM});
        public ObservableCollection<eProcessingType> SelectReconTypes
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

        

        private record PositionInfos(Vector3Collection V3, Color4Collection C4, List<double> Values);
        private PositionInfos RealtimePositionInfo = new PositionInfos(new Vector3Collection(), new Color4Collection(), new List<double>());

        private Task RealTimeImageReconTaskAsync;
        private void RealTimeImageRecon()
        {
            while (IsRealTimeImageReconOn)
            {
                var poseVect = new List<double[]>();
                var colorVect = new List<double[]>();
                var values = new List<double>();
                double maxValue = 0;

                LahgiWrapper_Static.ContinueReconPointFor1m1m1m(ref poseVect, ref values, ref maxValue);
              
                var vc = new Vector3Collection();
                var cc = new Color4Collection();
                for (int i = 0; i < poseVect.Count; i++)
                {
                    vc.Add(new Vector3(Convert.ToSingle(poseVect[i][0]), Convert.ToSingle(poseVect[i][1]), Convert.ToSingle(poseVect[i][2])));                    
                    cc.Add(ImageRecon.ColorScaleJet((float)values[i], (float)projectionMinValue, (float)projectionMaxValue));
                }
                RealtimePositionInfo = new PositionInfos(vc, cc, values);
                Application.Current.Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                {
                    TrueReconMaxValue = maxValue;
                    ProjectionMaxValue = maxValue;
                    UpdatePointCloudProjection();
                }));

                //
                Thread.Sleep(100);
            }
        }
        
        

        public record ProjectionInfo(double X, double Y, double Value, SolidColorBrush Color);        

        private List<ProjectionInfo> projectionPositions = new List<ProjectionInfo>();
        public List<ProjectionInfo> ProjectionPositions
        {
            get { return projectionPositions; }
            set { projectionPositions = value; OnPropertyChanged(nameof(ProjectionPositions)); }
        }

        public record ProjectionLineInfo(double X, double Y);
        private List<ProjectionLineInfo> projectionYLine = new List<ProjectionLineInfo>();
        public List<ProjectionLineInfo> ProjectionYLine
        {
            get { return projectionYLine; }
            set { projectionYLine = value; OnPropertyChanged(nameof(ProjectionYLine)); }
        }

        private List<ProjectionLineInfo> projectionXLine = new List<ProjectionLineInfo>();
        public List<ProjectionLineInfo> ProjectionXLine
        {
            get { return projectionXLine; }
            set { projectionXLine = value; OnPropertyChanged(nameof(ProjectionXLine)); }
        }

        private void UpdatePointCloudProjection()
        {
            var vc = new Vector3Collection();
            var cc = new Color4Collection();
            double projectionXRounded = Math.Round(projectionX, 2);
            double projectionYRounded = Math.Round(projectionY, 2);
            double projectionZRounded = Math.Round(projectionZ, 2);

            List<ProjectionInfo> tempProjection = new List<ProjectionInfo>();

            double maxPointX = double.MinValue;
            double maxPointY = double.MinValue;
            double maxPointZ = double.MinValue;
            double maxValue = double.MinValue;
            switch (SelectReconType)
            {
                case eProcessingType.SimpleBP:
                    for (int i = 0; i < RealtimePositionInfo.V3.Count(); ++i)
                    {
                        double x = Math.Round(RealtimePositionInfo.V3[i].X, 2);
                        double y = Math.Round(RealtimePositionInfo.V3[i].Y, 2);
                        double z = Math.Round(RealtimePositionInfo.V3[i].Z, 2);
                        double value = RealtimePositionInfo.Values[i];

                        if (x == projectionXRounded || y == projectionYRounded || z == projectionZRounded)
                        {
                            vc.Add(RealtimePositionInfo.V3[i]);
                            cc.Add(ImageRecon.ColorScaleJet((float)value, (float)projectionMinValue, (float)projectionMaxValue));
                            switch (ReconProjection)
                            {
                                case EReconProjection.XY:

                                    if (z == projectionZRounded)
                                    {
                                        var projectInfo = new ProjectionInfo(x, y, value, ColorScaleJetSolid(Convert.ToSingle(value), Convert.ToSingle(ProjectionMinValue), Convert.ToSingle(ProjectionMaxValue)));
                                        tempProjection.Add(projectInfo);
                                    }
                                    break;
                                case EReconProjection.XZ:
                                    if (y == projectionYRounded)
                                    {
                                        var projectInfo = new ProjectionInfo(x, z, value, ColorScaleJetSolid(Convert.ToSingle(value), Convert.ToSingle(ProjectionMinValue), Convert.ToSingle(ProjectionMaxValue)));
                                        tempProjection.Add(projectInfo);
                                    }
                                    break;
                                case EReconProjection.ZY:
                                    if (x == projectionXRounded)
                                    {
                                        var projectInfo = new ProjectionInfo(z, y, value, ColorScaleJetSolid(Convert.ToSingle(value), Convert.ToSingle(ProjectionMinValue), Convert.ToSingle(ProjectionMaxValue)));
                                        tempProjection.Add(projectInfo);
                                    }
                                    break;
                            }
                        }
                        
                        if (maxValue > value)
                        {
                            maxValue = value;
                            maxPointX = x;
                            maxPointY = y;
                            maxPointZ = z;
                        }
                    
                    }

                    break;
                case eProcessingType.MLEM:
                    break;
            }
            switch (ReconProjection)
            {
                case EReconProjection.XY:
                    {
                        ProjectionLineInfo xLine1 = new ProjectionLineInfo(projectionXRounded, -50);
                        ProjectionLineInfo xLine2 = new ProjectionLineInfo(projectionXRounded, +50);
                        ProjectionLineInfo yLine1 = new ProjectionLineInfo(-50, projectionYRounded);
                        ProjectionLineInfo yLine2 = new ProjectionLineInfo(+50, projectionYRounded);
                        List<ProjectionLineInfo> tmpX = new List<ProjectionLineInfo>() { xLine1, xLine2 };
                        List<ProjectionLineInfo> tmpY = new List<ProjectionLineInfo>() { yLine1, yLine2 };
                        ProjectionXLine = tmpX;
                        ProjectionYLine = tmpY;

                    }
                    break;
                case EReconProjection.XZ:
                    {
                        ProjectionLineInfo xLine1 = new ProjectionLineInfo(projectionXRounded, 0);
                        ProjectionLineInfo xLine2 = new ProjectionLineInfo(projectionXRounded, +100);
                        ProjectionLineInfo yLine1 = new ProjectionLineInfo(-50, projectionZRounded);
                        ProjectionLineInfo yLine2 = new ProjectionLineInfo(+50, projectionZRounded);
                        List<ProjectionLineInfo> tmpX = new List<ProjectionLineInfo>() { xLine1, xLine2 };
                        List<ProjectionLineInfo> tmpY = new List<ProjectionLineInfo>() { yLine1, yLine2 };
                        ProjectionXLine = tmpX;
                        ProjectionYLine = tmpY;
                    }
                    break;
                case EReconProjection.ZY:
                    {
                        ProjectionLineInfo xLine1 = new ProjectionLineInfo(projectionZRounded, -50);
                        ProjectionLineInfo xLine2 = new ProjectionLineInfo(projectionZRounded, +50);
                        ProjectionLineInfo yLine1 = new ProjectionLineInfo(0, projectionYRounded);
                        ProjectionLineInfo yLine2 = new ProjectionLineInfo(+100, projectionYRounded);
                        List<ProjectionLineInfo> tmpX = new List<ProjectionLineInfo>() { xLine1, xLine2 };
                        List<ProjectionLineInfo> tmpY = new List<ProjectionLineInfo>() { yLine1, yLine2 };
                        ProjectionXLine = tmpX;
                        ProjectionYLine = tmpY;
                    }
                    break;
            }

            ProjectionPositions = tempProjection;

           
            if(maxPointX == double.MinValue)
            {
                maxPointX = double.NaN;
            }
            if (maxPointY == double.MinValue)
            {
                maxPointY = double.NaN;
            }
            if (maxPointZ == double.MinValue)
            {
                maxPointZ = double.NaN;
            }

            MaxReconPointX = maxPointX;
            MaxReconPointY = maxPointY;
            MaxReconPointZ = maxPointZ;

            RTPointCloud = new PointGeometry3D() { Positions = vc, Colors = cc };            
        
        }


        private double maxReconPointX = 0;
        public double MaxReconPointX
        {
            get
            {
                return maxReconPointX;
            }
            set
            {
                maxReconPointX = value;
                OnPropertyChanged(nameof(MaxReconPointX));
            }
        }

        private double maxReconPointY = 0;
        public double MaxReconPointY
        {
            get
            {
                return maxReconPointY;
            }
            set
            {
                maxReconPointY = value;
                OnPropertyChanged(nameof(MaxReconPointY));
            }
        }

        private double maxReconPointZ = 0.5;
        public double MaxReconPointZ
        {
            get
            {
                return maxReconPointZ;
            }
            set
            {
                maxReconPointZ = value;
                OnPropertyChanged(nameof(MaxReconPointZ));
            }
        }
        #endregion

        private string postProcessingFileName;
        public string PostProcessingFileName
        {
            get
            {
                return postProcessingFileName;
            }
            set
            {
                postProcessingFileName = value;
                OnPropertyChanged(nameof(PostProcessingFileName));
            }
        }


        private RelayCommand postProcessingCommand;
        public ICommand PostProcessingCommnd
        {
            get { return (this.postProcessingCommand) ?? (this.postProcessingCommand = new RelayCommand(PostProcessing, CanExecutePostProcessingCommand)); }
        }
        private async void PostProcessing()
        {
            IsPostProcessingDone = false;
            
            Calcimg.MatPostpro matPostpro = new Calcimg.MatPostpro();
            //matPostpro.Calcimg();
            IsPostProcessingDone = true;
        }
        private bool isPostProcessingDone = true;
        public bool IsPostProcessingDone
        {
            get
            {
                return isPostProcessingDone;
            }
            set
            {
                isPostProcessingDone = value;
                OnPropertyChanged(nameof(IsPostProcessingDone));
                Application.Current.Dispatcher.Invoke(
                  DispatcherPriority.Normal,
                  new Action(() =>
                  {
                      ((RelayCommand)PostProcessingCommnd).RaiseCanExecuteChanged();
                  }));

            }
        }

        private bool CanExecutePostProcessingCommand()
        {          
            return IsPostProcessingDone || !IsSessionStart;
        }

        /// <summary>
        /// Hit map color scale
        /// </summary>
        /// <param name="idx"></param>
        /// <param name="minimum"></param>
        /// <param name="maximum"></param>
        /// <returns></returns>
        public static SolidColorBrush ColorScaleJetSolid(float v, float vmin, float vmax)
        {
            float dv;

            if (v < vmin)
            {
                v = vmin;
            }
            if (v > vmax)
            {
                v = vmax;
            }
            
            dv = vmax - vmin;

            float alpha = 1.0f;

            float r = 1.0f, g = 1.0f, b = 1.0f;
            if (dv == 0)
            {
                var colorWhite = System.Windows.Media.Color.FromScRgb(alpha, r, g, b);
                return new SolidColorBrush(colorWhite);
            }
            if (v < (vmin + 0.25f * dv))
            {
                r = 0;
                g = 4.0f * (v - vmin) / dv;
            }
            else if (v < (vmin + 0.5 * dv))
            {
                r = 0;
                b = 1 + 4.0f * (vmin + 0.25f * dv - v) / dv;
            }
            else if (v < (vmin + 0.75 * dv))
            {
                r = 4.0f * (v - vmin - 0.5f * dv) / dv;
                b = 0;
            }
            else
            {
                g = 1.0f + 4.0f * (vmin + 0.75f * dv - v) / dv;
                b = 0;
            }

            var color = System.Windows.Media.Color.FromScRgb(alpha, r, g, b);
            return new SolidColorBrush(color);
        }

    }
}
