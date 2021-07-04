using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL.Compton
{

    public struct ReconImagePoint
    {
        Vector3 CmagePoint;
        Color4 Color;
        double ImageValue;
    }

    public static class ComptonMatlabWrapper
    {
        public static bool MLEMRecon(string fileName, uint iterationTime, ref List<ReconImagePoint> imageSpace)
        {
      
            
            return false;
        }

        public static bool Radioactivity(string fileName, ref double radioAcitivity)
        {
            return false;
        }        
    }
}
