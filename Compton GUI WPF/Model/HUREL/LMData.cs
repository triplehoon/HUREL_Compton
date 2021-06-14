using System;
using System.Collections.Generic;
using System.Text;
using System.Windows.Media.Media3D;


namespace HUREL.Compton
{
    public class LMData
    {
        public record LMDataInfo(Point3D RelativeInteractionPoint3D, Point3D TransformedInteractionPoint3D, double InteractionEnergy);
        public enum InteractionType
        {
            CodedApature,
            Compton,
            Uncertain
        }

        /// <summary>
        /// Mesurement Time when LMData Constructed
        /// </summary>
        public DateTime MeasurementTime { get; init; }
        public InteractionType Type { get; init; }
       
        public Matrix3D DeviceTransformMatrix { get; init; }

        public LMDataInfo ScatterLMDataInfo { get; init; }                    
        public LMDataInfo AbsorberLMDataInfo { get; init; }

        private static List<double> TotalScatterInteractionEnergys = new List<double>();

        private static List<double> TotalAbsorberInteractionEnergys = new List<double>();

        #region Constructors

        public LMData(Point3D scatterPoint, double scatterEnergy)
        {
            MeasurementTime = DateTime.Now;
            Type = InteractionType.CodedApature;
            DeviceTransformMatrix = Matrix3D.Identity;
            ScatterLMDataInfo = new LMDataInfo(scatterPoint, scatterPoint, scatterEnergy);
            AbsorberLMDataInfo = null;
        }

        public LMData(Point3D scatterPoint, Point3D absorberPoint, double scatterEnergy, double absorberEnergy)
        {
            MeasurementTime = DateTime.Now;
            Type = InteractionType.Compton;
            DeviceTransformMatrix = Matrix3D.Identity;
            ScatterLMDataInfo = new LMDataInfo(scatterPoint, scatterPoint, scatterEnergy);
            AbsorberLMDataInfo = new LMDataInfo(absorberPoint, absorberPoint, absorberEnergy);
        }
        
        public LMData(Point3D scatterPoint, double scatterEnergy, Matrix3D tranformation)
        {            
            MeasurementTime = DateTime.Now;
            Type = InteractionType.CodedApature;
            DeviceTransformMatrix = tranformation;
            ScatterLMDataInfo = new LMDataInfo(scatterPoint, tranformation.Transform(scatterPoint), scatterEnergy);
            AbsorberLMDataInfo = null;
        }

        public LMData(Point3D scatterPoint, Point3D absorberPoint, double scatterEnergy, double absorberEnergy, Matrix3D tranformation)
        {
            MeasurementTime = DateTime.Now;
            Type = InteractionType.Compton;
            DeviceTransformMatrix = tranformation;
            ScatterLMDataInfo = new LMDataInfo(scatterPoint, tranformation.Transform(scatterPoint), scatterEnergy);
            AbsorberLMDataInfo = new LMDataInfo(absorberPoint, tranformation.Transform(absorberPoint), absorberEnergy);
        }       
        #endregion

        public static void Reset()
        {
            TotalScatterInteractionEnergys.Clear();
            TotalAbsorberInteractionEnergys.Clear();
        }

    }



}
