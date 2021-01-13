using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Media.Media3D;


namespace HUREL.Compton
{
    public class LMData
    {
        /// <summary>
        /// Mesurement Time when LMData Constructed
        /// </summary>
        public DateTime MeasurementTime { get; }

        public record LMDataInfo(Point3D TransformedInteractionPoint3D, double InteractionEnergy);       

        public enum InteractionType
        {
            CodedApature,
            Compton,
            Uncertain

        }

        private InteractionType type = InteractionType.Uncertain;
        public InteractionType Type
        {
            get
            {
                if (type != InteractionType.Uncertain)
                    return type;
                else
                {
                    if (ScatterInteractionCount == 0)
                        return InteractionType.Uncertain;
                    else if (AbsorberInteractionCount == 0)
                        type = InteractionType.CodedApature;
                    else
                        type = InteractionType.Compton;
                    return type;
                }
            }
        }

        private Matrix3D deviceTransformMatrix;
        public Matrix3D DeviceTransformMatrix
        {
            get { return deviceTransformMatrix; }
            set 
            {
                deviceTransformMatrix = value;
                scatterLMDataInfos = null;
                absorberLMDataInfos = null;
            }
        }

        #region Scatter

        private List<LMDataInfo> scatterLMDataInfos = null;
        public List<LMDataInfo> ScatterLMDataInfos
        {
            get 
            {
                if (scatterLMDataInfos != null)
                    return scatterLMDataInfos;
                else
                {
                    scatterLMDataInfos = new List<LMDataInfo>();

                    for (int i = 0; i < ScatterInteractionCount; i++)
                    {
                        var tempLMDataInfo = new LMDataInfo(DeviceTransformMatrix.Transform(ScatterInteractionPoint3Ds[i]), ScatterInteractionEnergys[i]) ;
                        scatterLMDataInfos.Add(tempLMDataInfo);
                    }
                    return scatterLMDataInfos;
                }
            }
        }

        /// <summary>
        /// Scatter Interaction Count
        /// </summary>
        private int ScatterInteractionCount { get { return ScatterInteractionPoint3Ds.Count; } }

        /// <summary>
        /// Scatter Interaction Point in Liset
        /// </summary>
        private List<Point3D> ScatterInteractionPoint3Ds { get; init; }

        /// <summary>
        /// Scatter Energy in keV
        /// </summary>
        private List<double> ScatterInteractionEnergys { get; init; }

        /// <summary>
        /// Total Scatter Interaction Count
        /// </summary>
        private static int TotalScatterInteractionCount = 0;

        /// <summary>
        /// Total Scater Energy List
        /// </summary>
        private static List<double> TotalScatterInteractionEnergys = new List<double>();
        #endregion

        #region Abosrber


        private List<LMDataInfo> absorberLMDataInfos = null;
        public List<LMDataInfo> AbsorberLMDataInfos
        {
            get
            {
                if (absorberLMDataInfos != null)
                    return absorberLMDataInfos;
                else
                {
                    absorberLMDataInfos = new List<LMDataInfo>();

                    for (int i = 0; i < AbsorberInteractionCount; i++)
                    {
                        var tempLMDataInfo = new LMDataInfo (DeviceTransformMatrix.Transform(AbsorberInteractionPoint3Ds[i]),AbsorberInteractionEnergys[i] );
                        absorberLMDataInfos.Add(tempLMDataInfo);
                    }
                    return absorberLMDataInfos;
                }
            }
        }

        /// <summary>
        /// Abosober Interaction Count
        /// </summary>
        private int AbsorberInteractionCount { get { return AbsorberInteractionPoint3Ds.Count; } }

        /// <summary>
        /// Abosober Interaction Point in List
        /// </summary>
        private List<Point3D> AbsorberInteractionPoint3Ds { get; init; }

        /// <summary>
        /// Abosober Energy in keV
        /// </summary>
        private List<double> AbsorberInteractionEnergys { get; init; }

        /// <summary>
        /// Total Abosober Interaction Count
        /// </summary>
        private static int TotalAbsorberInteractionCount = 0;
        private static List<double> TotalAbsorberInteractionEnergys = new List<double>();

        #endregion


        #region Constructors

        public LMData(Point3D scatterPoint, double scatterEnergy)
        {
            MeasurementTime = DateTime.Now;


            var scatterPoints = new List<Point3D>
            {
                scatterPoint
            };
            ScatterInteractionPoint3Ds = scatterPoints;

            var scatterEnergys = new List<double>
            {
                scatterEnergy
            };
            TotalScatterInteractionEnergys.Add(scatterEnergy);
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionCount++;


            var absorberPoints = new List<Point3D>();
            AbsorberInteractionPoint3Ds = absorberPoints;

            var absorberEnergys = new List<double>();
            AbsorberInteractionEnergys = absorberEnergys;


            DeviceTransformMatrix = Matrix3D.Identity;

            TotalAbsorberInteractionCount++;
        }

        public LMData(Point3D scatterPoint, Point3D absorberPoint, double scatterEnergy, double absorberEnergy)
        {
            MeasurementTime = DateTime.Now;

            var scatterPoints = new List<Point3D>
            {
                scatterPoint
            };
            ScatterInteractionPoint3Ds = scatterPoints;

            var scatterEnergys = new List<double>
            {
                scatterEnergy
            };
            TotalScatterInteractionEnergys.Add(scatterEnergy);
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionCount++;


            var absorberPoints = new List<Point3D>
            {
                absorberPoint
            };
            AbsorberInteractionPoint3Ds = absorberPoints;

            var absorberEnergys = new List<double>();
            absorberEnergys.Add(absorberEnergy);
            TotalScatterInteractionEnergys.Add(absorberEnergy);
            AbsorberInteractionEnergys = absorberEnergys;

            DeviceTransformMatrix = Matrix3D.Identity;

            TotalAbsorberInteractionCount++;
        }

        public LMData(IEnumerable<Point3D> scatterPoint, IEnumerable<Point3D> absorberPoint, IEnumerable<double> scatterEnergy, IEnumerable<double> absorberEnergy)
        {
            MeasurementTime = DateTime.Now;

            var scatterPoints = new List<Point3D>();
            scatterPoints.AddRange(scatterPoint);
            ScatterInteractionPoint3Ds = scatterPoints;

            var scatterEnergys = new List<double>();
            scatterEnergys.AddRange(scatterEnergy);
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionEnergys.AddRange(scatterEnergys);
            TotalScatterInteractionCount += ScatterInteractionCount;

            var absorberPoints = new List<Point3D>();
            absorberPoints.AddRange(absorberPoint);
            AbsorberInteractionPoint3Ds = absorberPoints;

            var absorberEnergys = new List<double>();
            absorberEnergys.AddRange(absorberEnergy);
            AbsorberInteractionEnergys = absorberEnergys;

            TotalAbsorberInteractionEnergys.AddRange(absorberEnergys);
            TotalAbsorberInteractionCount += AbsorberInteractionCount;

            DeviceTransformMatrix = Matrix3D.Identity;

        }

        public LMData(Point3D scatterPoint, double scatterEnergy, Matrix3D tranformation)
        {
            MeasurementTime = DateTime.Now;


            var scatterPoints = new List<Point3D>
            {
                scatterPoint
            };
            ScatterInteractionPoint3Ds = scatterPoints;

            var scatterEnergys = new List<double>
            {
                scatterEnergy
            };
            TotalScatterInteractionEnergys.Add(scatterEnergy);
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionCount++;


            var absorberPoints = new List<Point3D>();
            AbsorberInteractionPoint3Ds = absorberPoints;

            var absorberEnergys = new List<double>();
            AbsorberInteractionEnergys = absorberEnergys;


            DeviceTransformMatrix = tranformation;

            TotalAbsorberInteractionCount++;
        }

        public LMData(Point3D scatterPoint, Point3D absorberPoint, double scatterEnergy, double absorberEnergy, Matrix3D tranformation)
        {
            MeasurementTime = DateTime.Now;

            var scatterPoints = new List<Point3D>
            {
                scatterPoint
            };
            ScatterInteractionPoint3Ds = scatterPoints;

            var scatterEnergys = new List<double>
            {
                scatterEnergy
            };
            TotalScatterInteractionEnergys.Add(scatterEnergy);
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionCount++;


            var absorberPoints = new List<Point3D>
            {
                absorberPoint
            };
            AbsorberInteractionPoint3Ds = absorberPoints;

            var absorberEnergys = new List<double>
            {
                absorberEnergy
            };
            TotalScatterInteractionEnergys.Add(absorberEnergy);
            AbsorberInteractionEnergys = absorberEnergys;

            DeviceTransformMatrix = tranformation;

            TotalAbsorberInteractionCount++;      
        }

        public LMData( IEnumerable<Point3D> scatterPoint, IEnumerable<Point3D> absorberPoint, IEnumerable<double> scatterEnergy, IEnumerable<double> absorberEnergy, Matrix3D tranformation)
        {
            MeasurementTime = DateTime.Now;

            var scatterPoints = new List<Point3D>();
            scatterPoints.AddRange(scatterPoint);
            ScatterInteractionPoint3Ds = scatterPoints;

            var scatterEnergys = new List<double>();
            scatterEnergys.AddRange(scatterEnergy);            
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionEnergys.AddRange(scatterEnergys);
            TotalScatterInteractionCount += ScatterInteractionCount;

            var absorberPoints = new List<Point3D>();
            absorberPoints.AddRange(absorberPoint);
            AbsorberInteractionPoint3Ds = absorberPoints;

            var absorberEnergys = new List<double>();
            absorberEnergys.AddRange(absorberEnergy);
            AbsorberInteractionEnergys = absorberEnergys;

            TotalAbsorberInteractionEnergys.AddRange(absorberEnergys);
            TotalAbsorberInteractionCount += AbsorberInteractionCount;

            DeviceTransformMatrix = tranformation;
 
        }

        public LMData(IEnumerable<(Point3D,double)> scatterMLPEdata, IEnumerable<(Point3D, double)> absorberMLPEdata, Matrix3D tranformation)
        {
            MeasurementTime = DateTime.Now;

            var scatterPoints = new List<Point3D>();
            var scatterEnergys = new List<double>();
            
            foreach(var item in scatterMLPEdata)
            {
                scatterPoints.Add(item.Item1);
                scatterEnergys.Add(item.Item2);
            }    


            ScatterInteractionPoint3Ds = scatterPoints;
            ScatterInteractionEnergys = scatterEnergys;

            TotalScatterInteractionEnergys.AddRange(scatterEnergys);
            TotalScatterInteractionCount += ScatterInteractionCount;

            var absorberPoints = new List<Point3D>();
            var absorberEnergys = new List<double>();

            foreach (var item in absorberMLPEdata)
            {
                absorberPoints.Add(item.Item1);
                absorberEnergys.Add(item.Item2);
            }

            AbsorberInteractionPoint3Ds = absorberPoints;
            AbsorberInteractionEnergys = absorberEnergys;

            TotalAbsorberInteractionEnergys.AddRange(absorberEnergys);
            TotalAbsorberInteractionCount += AbsorberInteractionCount;

            DeviceTransformMatrix = tranformation;

        }

        #endregion

        public static void Reset()
        {
            TotalScatterInteractionCount = 0;
            TotalAbsorberInteractionCount = 0;
            TotalScatterInteractionEnergys.Clear();
            TotalAbsorberInteractionEnergys.Clear();
        }

    }

    
    
}
