
using System.Windows.Media.Imaging;

namespace HUREL.Compton
{
    public static class LahgiApi
    {
        private static LahgiWrapper lahgiWrapper;
        private static RtabmapWrapper rtabmapWrapper;
        public static string StatusMsg = "";
        public static bool IsLahgiInitiate { get; private set; }
        public static bool IsRtabmapInitiate { get; private set; }
        public static bool IsInitiate
        {
            get
            {
                return IsLahgiInitiate && IsRtabmapInitiate;
            }
        }

        /// <summary>
        /// Start Stop Counting, Get Spectrum, Get 3D image, Get 2D image, 
        /// Start and stop imaging
        /// </summary>
        static LahgiApi()
        {
            lahgiWrapper = new LahgiWrapper();
            rtabmapWrapper = new RtabmapWrapper();
            InitiateLaghi();
            InititateRtabmap();
        }
        public static bool InitiateLaghi()
        {
            StatusMsg = "Initiating LAHGI";

            if (lahgiWrapper.Initiate(eModuleManagedType.QUAD))
            {
                StatusMsg = "Successfully initiate Lahgi";
                IsLahgiInitiate = true;
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Lahgi";
                IsLahgiInitiate = false;
                return false;
            }
        }
        public static bool InititateRtabmap()
        {

            StatusMsg = "Initiating RTABAMP";

            string msg = "";
            if (rtabmapWrapper.InitiateRtabmap(ref msg))
            {
                StatusMsg = "Successfully initiate Rtabmap";
                IsRtabmapInitiate = true;
                return true;
            }
            else
            {
                StatusMsg = "Fail to initiate Rtabmap";
                IsRtabmapInitiate = false;
                return false;
            }
        }
        public static bool StartRtabmap()
        {
            if (!IsRtabmapInitiate)
            {
                return false;
            }

            return rtabmapWrapper.StartRtabmapPipeline(ref StatusMsg);
        }
        public static void StopRtabmap()
        {
            rtabmapWrapper.StopRtabmapPipeline();
        }
  
        public static BitmapImage GetRgbImage()
        {
            BitmapImage img = null;

            return img
        }
    }
}