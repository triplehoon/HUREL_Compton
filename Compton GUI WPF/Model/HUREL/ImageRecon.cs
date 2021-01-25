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

            int xBinSize = Convert.ToInt32(Math.Ceiling(sizeX / voxelSize));
            int yBinSize = Convert.ToInt32(Math.Ceiling(sizeY / voxelSize));
            int zBinSize = Convert.ToInt32(Math.Ceiling(sizeZ / voxelSize));

            for (int x = 0; x < xBinSize; x++)
            {
                for (int y = 0; y < yBinSize; y++)
                {
                    for (int z = 0; z < zBinSize; z++)
                    {
                        imageSpace.Add(new Vector3()
                        {
                            X = x * voxelSize + minX,
                            Y = y * voxelSize + minY,
                            Z = z * voxelSize + minZ
                        });
                    }
                }
            }



            return imageSpace;
        }
        
        public static Tuple<Vector3Collection, Color4Collection> BPtoPointCloud(Vector3Collection imageSpace, List<LMData> lmDataList, double angleThreshold = 5)
        {
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (imageSpace.Count == 0 || lmDataList.Count == 0)
            {
                var tupleOut1 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut1;
            }
                
            int[] counts = new int[imageSpace.Count];
            var templmlist = lmDataList;


            foreach (var lmData in templmlist)
            {
                if (lmData.Type == LMData.InteractionType.Compton)
                {

                    for (int i = 0; i < imageSpace.Count; ++i)
                    {
                        if (IsEffectedPoint(lmData.ScatterLMDataInfos.First().RelativeInteractionPoint3D, lmData.ScatterLMDataInfos.First().InteractionEnergy,
                            lmData.AbsorberLMDataInfos.First().RelativeInteractionPoint3D, lmData.AbsorberLMDataInfos.First().InteractionEnergy,
                            imageSpace[i].ToPoint3D(), angleThreshold))
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


            for(int i = 0; i< imageSpace.Count; i++)
            {
                if (counts[i] > minimum)
                {
                    vector3sOut.Add(imageSpace[i]);
                    color4sOut.Add(ColorScale(counts[i], minimum, maximumCount));
                }
            }

            var tupleOut3 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
            return tupleOut3;

        }

        public static Tuple<Vector3Collection, Color4Collection> BPtoPointCloudSLAM(Vector3Collection imageSpace, List<LMData> lmDataList, double angleThreshold = 5)
        {
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (imageSpace.Count == 0 || lmDataList.Count == 0)
            {
                var tupleOut1 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
                return tupleOut1;
            }

            int[] counts = new int[imageSpace.Count];
            var templmlist = lmDataList;


            foreach (var lmData in templmlist)
            {
                if (lmData.Type == LMData.InteractionType.Compton)
                {

                    for (int i = 0; i < imageSpace.Count; ++i)
                    {
                        if (IsEffectedPoint(lmData.ScatterLMDataInfos.First().TransformedInteractionPoint3D, lmData.ScatterLMDataInfos.First().InteractionEnergy,
                            lmData.AbsorberLMDataInfos.First().TransformedInteractionPoint3D, lmData.AbsorberLMDataInfos.First().InteractionEnergy,
                            imageSpace[i].ToPoint3D(), angleThreshold))
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


            Parallel.For(0, imageSpace.Count, (i) =>
            {
                if (counts[i] > minimum)
                {
                    vector3sOut.Add(imageSpace[i]);
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
