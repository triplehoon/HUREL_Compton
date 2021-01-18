using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton;
using SharpDX;

namespace HUREL.Compton
{
    class Backprojection
    {
        private static bool IsEffectedPoint(Point3D scatterPhotonPosition, double scatterPhotonEnergy, 
            Point3D absorberPhotonPosition, double absorberPhotonEnergy, Point3D imgSpacePosition, double angleThreshold = 5)
        {
            double comptonCal = 1 - 511 * scatterPhotonEnergy / absorberPhotonEnergy / (scatterPhotonEnergy + absorberPhotonEnergy);
            if (comptonCal >= 1 || comptonCal <= -1)
                return false;

            double comptonScatteringAngle = Math.Acos(comptonCal) / Math.PI * 180;

            Vector3D effectToScatterVector = (scatterPhotonPosition - imgSpacePosition);
            Vector3D scatterToAbsorberVector = (absorberPhotonPosition - scatterPhotonPosition);
            effectToScatterVector.Normalize();
            scatterToAbsorberVector.Normalize();
            double positionDotPord = Vector3D.DotProduct(effectToScatterVector, scatterToAbsorberVector);

            double effectedAngle = Math.Acos(positionDotPord) / Math.PI * 180;

            if (Math.Abs(effectedAngle - comptonScatteringAngle) < angleThreshold)
                return true;
            else
                return false;
        }

        public static Tuple<Vector3Collection, Color4Collection> BPtoPointCloud(Vector3Collection vector3s, Color4Collection color4s, List<LMData> lmDataList)
        {
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (vector3s.Count == 0 || lmDataList.Count == 0)
            {
                var tupleOut1 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut1;
            }
                
            int[] counts = new int[vector3s.Count];
            var templmlist = lmDataList;


            foreach (var lmData in templmlist)
            {
                if (lmData.Type == LMData.InteractionType.Compton)
                {

                    for (int i = 0; i < vector3s.Count(); i=i+100)
                    {
                        if (IsEffectedPoint(lmData.ScatterLMDataInfos.First().RelativeInteractionPoint3D, lmData.ScatterLMDataInfos.First().InteractionEnergy,
                            lmData.AbsorberLMDataInfos.First().RelativeInteractionPoint3D, lmData.AbsorberLMDataInfos.First().InteractionEnergy,
                            vector3s[i].ToPoint3D(), 5))
                        {
                            counts[i]++;

                        }
                    }
                }
            }

           

                int maximumCount = counts.Max();
            int minimum = maximumCount*2/3;
            if (maximumCount < 5)
            {
                var tupleOut2 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut2;
            }


            for(int i = 0; i< vector3s.Count(); i++)
            {
                if (counts[i] > minimum)
                {
                    vector3sOut.Add(vector3s[i]);
                    color4sOut.Add(ColorScale(counts[i], minimum, maximumCount));
                }
            }

            var tupleOut3 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
            return tupleOut3;

        }

        public static Tuple<Vector3Collection, Color4Collection> BPtoPointCloudSLAM(Vector3Collection vector3s, Color4Collection color4s, List<LMData> lmDataList)
        {
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (vector3s.Count == 0 || lmDataList.Count == 0)
            {
                var tupleOut1 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut1;
            }

            int[] counts = new int[vector3s.Count];
            var templmlist = lmDataList;


            foreach (var lmData in templmlist)
            {
                if (lmData.Type == LMData.InteractionType.Compton)
                {

                    for (int i = 0; i < vector3s.Count(); i = i + 100)
                    {
                        if (IsEffectedPoint(lmData.ScatterLMDataInfos.First().TransformedInteractionPoint3D, lmData.ScatterLMDataInfos.First().InteractionEnergy,
                            lmData.AbsorberLMDataInfos.First().TransformedInteractionPoint3D, lmData.AbsorberLMDataInfos.First().InteractionEnergy,
                            vector3s[i].ToPoint3D(), 5))
                        {
                            counts[i]++;

                        }
                    }
                }
            }



            int maximumCount = counts.Max();
            int minimum = maximumCount * 2 / 3;
            if (maximumCount < 5)
            {
                var tupleOut2 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut2;
            }


            Parallel.For(0, vector3s.Count(), (i) =>
            {
                if (counts[i] > minimum)
                {
                    vector3sOut.Add(vector3s[i]);
                    color4sOut.Add(ColorScale(counts[i], minimum, maximumCount));
                }
            });

            var tupleOut3 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
            return tupleOut3;

        }





        private static Color4 ColorScale(int idx, int minimum, int maximum)
        {
            float alpha = 0.5f;
            if (idx < minimum+1)
                alpha = 0;
            Color4 color = new Color4(idx / (maximum ), (maximum - idx) / (maximum), 0.5f, alpha);
            return color;
        }

    }
}
