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
    public static class ImageRecon
    {       
        private static bool IsEffectedBPPoint(Point3D scatterPhotonPosition, double scatterPhotonEnergy, 
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


        /// <summary>
        /// Make Imaginary Image Space. Set voxelSize as dividable number (Recommanded)
        /// </summary>
        /// <param name="minX">[m]</param>
        /// <param name="maxX">[m]</param>
        /// <param name="minY">[m]</param>
        /// <param name="maxY">[m]</param>
        /// <param name="minZ">[m]</param>
        /// <param name="maxZ">[m]</param>
        /// <param name="voxelSize">[m]</param>
        /// <returns></returns>
        public static Vector3Collection GetImageSpaceByVoxel(float minX, float maxX, float minY, float maxY, float minZ, float maxZ, float voxelSize)
        {
            Vector3Collection imageSpace = new Vector3Collection();

            float sizeX = Convert.ToSingle(Math.Round(maxX - minX));
            float sizeY = Convert.ToSingle(Math.Round(maxY - minY));
            float sizeZ = Convert.ToSingle(Math.Round(maxZ - minZ));

            if(voxelSize > sizeX || voxelSize > sizeY || voxelSize > sizeZ)
            {
                ArgumentOutOfRangeException argumentOutOfRangeException = new ArgumentOutOfRangeException("Voxel Size is too large");
                throw argumentOutOfRangeException;
                
            }

            int xBinSize = Convert.ToInt32(Math.Floor(sizeX / voxelSize));
            int yBinSize = Convert.ToInt32(Math.Floor(sizeY / voxelSize));
            int zBinSize = Convert.ToInt32(Math.Floor(sizeZ / voxelSize));

            for (int x = 0; x < xBinSize; x++)
            {
                for (int y = 0; y < yBinSize; y++)
                {
                    for (int z = 0; z < zBinSize; z++)
                    {
                        imageSpace.Add(new Vector3()
                        {
                            X = (x + 1)* (voxelSize) + minX,
                            Y = (y + 1) * (voxelSize) + minY,
                            Z = (z + 1) * (voxelSize) + minZ
                        });
                    }
                }
            }



            return imageSpace;
        }
        
        public static Tuple<Vector3Collection, Color4Collection> BPtoPointCloud(Vector3Collection imageSpace, List<LMData> lmDataList, double angleThreshold = 5, double minCountPercent = 0)
        {
            if (minCountPercent > 1 || minCountPercent < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(minCountPercent));
            }
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (imageSpace.Count == 0 || lmDataList.Count == 0)
            {
                var tupleOut1 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut1;
            }
                
            int[] counts = new int[imageSpace.Count];
            var templmlist = lmDataList.ToArray();
            

            foreach (var lmData in templmlist)
            {
                if (lmData.Type == LMData.InteractionType.Compton)
                {
                    Parallel.For(0, imageSpace.Count, i =>
                    {
                         if (IsEffectedBPPoint(lmData.ScatterLMDataInfos[0].RelativeInteractionPoint3D, lmData.ScatterLMDataInfos[0].InteractionEnergy,
                            lmData.AbsorberLMDataInfos[0].RelativeInteractionPoint3D, lmData.AbsorberLMDataInfos[0].InteractionEnergy,
                            imageSpace[i].ToPoint3D(), angleThreshold))
                         {
                             counts[i]++;
                         }
                    });
                }
            }

           

            int maxCount = counts.Max();
            int minCount = Convert.ToInt32(Math.Round(maxCount * minCountPercent));
            if (maxCount < 5)
            {
                var tupleOut2 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut2;
            }


            for(int i = 0; i< imageSpace.Count; i++)
            {
                if (counts[i] > minCount)
                {
                    vector3sOut.Add(imageSpace[i]);
                    color4sOut.Add(ColorScaleJet(counts[i], minCount, maxCount));
                }
            }

            var tupleOut3 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
            return tupleOut3;

        }
        

        /// <summary>
        /// Hit map color scale
        /// </summary>
        /// <param name="idx"></param>
        /// <param name="minimum"></param>
        /// <param name="maximum"></param>
        /// <returns></returns>
        private static Color4 ColorScaleJet(float v, float vmin, float vmax)
        {
            float dv;

            if (v < vmin)
                v = vmin;
            if (v > vmax)
                v = vmax;
            dv = vmax - vmin;
            float r = 1.0f, g = 1.0f, b = 1.0f;
            if (v < (vmin + 0.25 * dv))
            {
                r = 0;
                g = 4 * (v - vmin) / dv;
            }
            else if (v < (vmin + 0.5 * dv))
            {
                r = 0;
                b = 1 + 4 * (vmin + 0.25f * dv - v) / dv;
            }
            else if (v < (vmin + 0.75 * dv))
            {
                r = 4 * (v - vmin - 0.5f * dv) / dv;
                b = 0;
            }
            else
            {
                g = 1 + 4 * (vmin + 0.75f * dv - v) / dv;
                b = 0;
            }

            float alpha = 0.5f;

            Color4 color = new Color4(r,g,b, alpha);
            return color;
        }
        
    } 
}
