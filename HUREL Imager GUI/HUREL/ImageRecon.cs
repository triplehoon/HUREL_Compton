using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;

namespace HUREL.Compton
{
    public class ImageRecon
    {
        private record ImagePoint(Point3D ImageSpace, int count, float u, float v);


        private static bool IsEffectedBPPoint(Point3D scatterPhotonPosition, double scatterPhotonEnergy,
           Point3D absorberPhotonPosition, double absorberPhotonEnergy, Point3D imgSpacePosition, double angleThreshold = 5)
        {
            double comptonCal = 1 - 511 * scatterPhotonEnergy / absorberPhotonEnergy / (scatterPhotonEnergy + absorberPhotonEnergy);
            if (comptonCal >= 1 || comptonCal <= -1)
                return false;
            double comptonScatteringAngle = Math.Acos(comptonCal) / Math.PI * 180;                       
            Vector3D effectToScatterVector = (imgSpacePosition - scatterPhotonPosition);
            Vector3D scatterToAbsorberVector = (scatterPhotonPosition - absorberPhotonPosition);
            effectToScatterVector.Normalize();
            scatterToAbsorberVector.Normalize();
            double positionDotPord = Vector3D.DotProduct(effectToScatterVector, scatterToAbsorberVector);
           
            double effectedAngle = Math.Acos(positionDotPord) / Math.PI * 180;

            if (Math.Abs(effectedAngle - comptonScatteringAngle) < angleThreshold)
                return true;
            else
                return false;
        }


        private static bool IsEffectedBPPoint2Pi(Point3D scatterPhotonPosition, double scatterPhotonEnergy,
           Point3D absorberPhotonPosition, double absorberPhotonEnergy, Point3D imgSpacePosition, Matrix3D transformM, double angleThreshold = 5)
        {
            var frontVector = (Vector3D)transformM.Transform(new Point3D(0, 0, 1));
            Vector3D effectToScatterVector = imgSpacePosition - scatterPhotonPosition;
            Vector3D scatterToAbsorberVector = scatterPhotonPosition - absorberPhotonPosition;
            if (Vector3D.DotProduct(frontVector, effectToScatterVector) < 0) 
            {
                return false;
            }

            double comptonCal = 1 - 511 * scatterPhotonEnergy / absorberPhotonEnergy / (scatterPhotonEnergy + absorberPhotonEnergy);
            if (comptonCal >= 1 || comptonCal <= -1)
                return false;
            double comptonScatteringAngle = Math.Acos(comptonCal) / Math.PI * 180;

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

            if (voxelSize > sizeX || voxelSize > sizeY || voxelSize > sizeZ)
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
                            X = (x + 1) * (voxelSize) + minX,
                            Y = (y + 1) * (voxelSize) + minY,
                            Z = (z + 1) * (voxelSize) + minZ
                        });
                    }
                }
            }



            return imageSpace;
        } 
        
        public static (Vector3Collection, Color4Collection, Bitmap) BPtoPointCloudBitmap(Vector3Collection imageSpace, List<float[]> uvs, List<LMData> lmDataList, int height, int width, bool isTransFormed = false, double angleThreshold = 5, double minCountPercent = 0)
        {
            if (minCountPercent > 1 || minCountPercent < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(minCountPercent));
            }
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();
            Bitmap bitmapOut = new Bitmap(1, 1);
            if (imageSpace.Count == 0 || lmDataList.Count == 0 || height == 0 || width == 0)
            {                
                return (vector3sOut, color4sOut, bitmapOut);
            }

            bitmapOut = new Bitmap(width, height);

            int[] counts = new int[imageSpace.Count];

            if (isTransFormed)
            {
                foreach (var lmData in lmDataList)
                {
                    if (lmData.Type == LMData.InteractionType.Compton)
                    {
                        Parallel.For(0, imageSpace.Count, i =>// for (int i = 0; i < imageSpace.Count; ++i)
                        {
                            if (IsEffectedBPPoint(lmData.ScatterLMDataInfo.TransformedInteractionPoint3D, lmData.ScatterLMDataInfo.InteractionEnergy,
                               lmData.AbsorberLMDataInfo.TransformedInteractionPoint3D, lmData.AbsorberLMDataInfo.InteractionEnergy,
                               imageSpace[i].ToPoint3D(), angleThreshold))
                            {
                                counts[i]++;
                            }
                        });
                    }
                }
            }
            else
            {
                foreach (var lmData in lmDataList)
                {
                    if (lmData.Type == LMData.InteractionType.Compton)
                    {
                        for (int i = 0; i < imageSpace.Count; ++i)
                        {
                            if (IsEffectedBPPoint(lmData.ScatterLMDataInfo.RelativeInteractionPoint3D, lmData.ScatterLMDataInfo.InteractionEnergy,
                               lmData.AbsorberLMDataInfo.RelativeInteractionPoint3D, lmData.AbsorberLMDataInfo.InteractionEnergy,
                               imageSpace[i].ToPoint3D(), angleThreshold))
                            {
                                counts[i]++;
                            }
                        }
                    }
                }
            }


            int maxCount = counts.Max();
            int minCount = Convert.ToInt32(Math.Round(maxCount * minCountPercent));
            if (maxCount < 5)
            {
                return (vector3sOut, color4sOut, bitmapOut);
            }


            for (int i = 0; i < imageSpace.Count; i++)
            {
                if (counts[i] > minCount)
                {
                    vector3sOut.Add(imageSpace[i]);
                    Color4 jetColor = ColorScaleJet(counts[i], minCount, maxCount);

                    System.Drawing.Color bitMapColor = System.Drawing.Color.FromArgb(Convert.ToInt32(jetColor.Alpha * 255),
                        Convert.ToInt32(jetColor.Red * 255),
                        Convert.ToInt32(jetColor.Green * 255),
                        Convert.ToInt32(jetColor.Blue * 255));
                    color4sOut.Add(jetColor);

                    int u = (int)Math.Round(uvs[i][0] * width);
                    int v = (int)Math.Round(uvs[i][1] * height);
                    if (u == width)
                    {
                        --u;
                    }
                    if (v == height)
                    {
                        --v;
                    }

                    bitmapOut.SetPixel(u, v, bitMapColor);                       
                }               
            }

            var tupleOut3 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
            return (vector3sOut, color4sOut, bitmapOut);


        }
        
        public static (Vector3Collection, Color4Collection) BPtoPointCloud(Vector3Collection imageSpace, List<LMData> lmDataList, bool isTransFormed = false, double angleThreshold = 5, double minCountPercent = 0)
        {
            if (minCountPercent > 1 || minCountPercent < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(minCountPercent));
            }
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (imageSpace.Count == 0 || lmDataList.Count == 0)
            {
                return (vector3sOut, color4sOut);
            }

            int[] counts = new int[imageSpace.Count];

            if (isTransFormed)
            {
                foreach (var lmData in lmDataList)
                {
                    if (lmData.Type == LMData.InteractionType.Compton)
                    {                       
                        for (int i = 0; i < imageSpace.Count;++i)
                        {
                            if (IsEffectedBPPoint(lmData.ScatterLMDataInfo.TransformedInteractionPoint3D, lmData.ScatterLMDataInfo.InteractionEnergy,
                               lmData.AbsorberLMDataInfo.TransformedInteractionPoint3D, lmData.AbsorberLMDataInfo.InteractionEnergy,
                               imageSpace[i].ToPoint3D(), angleThreshold))
                            {
                                counts[i]++;
                            }
                        }
                    }
                }
            }
            else
            {
                foreach (var lmData in lmDataList)
                {
                    if (lmData.Type == LMData.InteractionType.Compton)
                    {
                        for (int i = 0; i < imageSpace.Count; ++i)
                        {
                            if (IsEffectedBPPoint(lmData.ScatterLMDataInfo.RelativeInteractionPoint3D, lmData.ScatterLMDataInfo.InteractionEnergy,
                               lmData.AbsorberLMDataInfo.RelativeInteractionPoint3D, lmData.AbsorberLMDataInfo.InteractionEnergy,
                               imageSpace[i].ToPoint3D(), angleThreshold))
                            {
                                counts[i]++;
                            }
                        }
                    }
                }
            }

            int maxCount = counts.Max();
            int minCount = Convert.ToInt32(Math.Round(maxCount * minCountPercent));
            if (maxCount < 5)
            {
                return (vector3sOut, color4sOut);
            }


            for (int i = 0; i < imageSpace.Count; i++)
            {
                if (counts[i] > minCount)
                {
                    vector3sOut.Add(imageSpace[i]);
                    color4sOut.Add(ColorScaleJet(counts[i], minCount, maxCount));
                }
            }

            var tupleOut3 = new Tuple<Vector3Collection, Color4Collection>(vector3sOut, color4sOut);
            return (vector3sOut, color4sOut);

        }

        public static (Vector3Collection, Color4Collection) BPtoPointCloud2Pi(Vector3Collection imageSpace, List<LMData> lmDataList, double angleThreshold = 5, double minCountPercent = 0)
        {
            if (minCountPercent > 1 || minCountPercent < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(minCountPercent));
            }
            Vector3Collection vector3sOut = new Vector3Collection();
            Color4Collection color4sOut = new Color4Collection();

            if (imageSpace.Count == 0 || lmDataList.Count == 0)
            {
                return (vector3sOut, color4sOut);
            }

            int[] counts = new int[imageSpace.Count];


            foreach (var lmData in lmDataList)
            {
                if (lmData.Type == LMData.InteractionType.Compton)
                {                     
                    for (int i = 0; i < imageSpace.Count; ++i)
                    {
                        if (IsEffectedBPPoint2Pi(lmData.ScatterLMDataInfo.TransformedInteractionPoint3D, lmData.ScatterLMDataInfo.InteractionEnergy,
                           lmData.AbsorberLMDataInfo.TransformedInteractionPoint3D, lmData.AbsorberLMDataInfo.InteractionEnergy,
                           imageSpace[i].ToPoint3D(), lmData.DeviceTransformMatrix, angleThreshold))
                        {
                            counts[i]++;
                        }
                    }
                }
            }



            int maxCount = counts.Max();
            int minCount = Convert.ToInt32(Math.Round(maxCount * minCountPercent));
            if (maxCount < 5)
            {
                return (vector3sOut, color4sOut);
            }


            for (int i = 0; i < imageSpace.Count; i++)
            {
                if (counts[i] > minCount)
                {
                    vector3sOut.Add(imageSpace[i]);
                    color4sOut.Add(ColorScaleJet(counts[i], minCount, maxCount));
                }
            }

            return (vector3sOut, color4sOut);

        }

        public static (Vector3Collection, List<float[]>) GetImageSpaceBySurfaceFOV(int rgbWidth, int rgbHeight, double hfov, double vfov, double distance)
        {
            Vector3Collection vector3s = new Vector3Collection();
            List<float[]> uvs = new List<float[]>();
            double hfovRad = hfov * Math.PI / 180 ;
            double vfovRad = vfov * Math.PI / 180 ;
            for (int u = 0; u < rgbWidth; ++u)
            {
                for (int v = 0; v < rgbHeight; ++v)
                {
                    double theta = hfovRad / 2 - hfovRad * u / rgbWidth;
                    double omega = vfovRad / 2 - vfovRad * v / rgbHeight;
                    vector3s.Add(new Vector3((float)(distance * Math.Cos(omega) * Math.Sin(theta)), (float)(distance * Math.Sin(omega)), (float)(distance * Math.Cos(omega) * Math.Cos(theta))));
                    uvs.Add(new float[2] { (float)u / (float)rgbWidth, (float)v / (float)rgbHeight });
                }
            }

            return (vector3s, uvs);
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
            {
                v = vmin;
            }
            if (v > vmax)
            {
                v = vmax;
            }
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

            float alpha = 0.8f;
           
            var color = new Color4(r, g, b, alpha);
            
            return color;
        }

    } 
}
