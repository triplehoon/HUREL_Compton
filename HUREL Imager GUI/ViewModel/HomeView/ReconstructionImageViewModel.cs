using HUREL.Compton;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;

namespace HUREL_Imager_GUI.ViewModel
{
    public class ReconstructionImageViewModel: ViewModelBase
    {
        public ReconstructionImageViewModel()
        {
            realtimeRGB = new BitmapImage();
            LoopTask = Task.Run(Loop);
            codedImgRGB = new BitmapImage();
            comptonImgRGB = new BitmapImage();
            hybridImgRGB = new BitmapImage();

            LahgiApi.StatusUpdate += StatusUpdate;
        }

        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }
            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Status || lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.SlamRadImage)
                {
                    BitmapImage? tmpCode;
                    BitmapImage? tmpCompton;
                    BitmapImage? tmpHybrid;
                    
                    //848 480 90 60    51 90  48 85
                    (tmpCode, tmpCompton, tmpHybrid) = LahgiApi.GetRadation2dImage(timeInMiliSeconds, s2M, det_W, resImprov, m2D, 60, 90, imgSize, minValuePortion);

                    if (tmpCode == null || tmpCompton == null || tmpHybrid == null)
                    {
                        StatusUpdateMutex.ReleaseMutex();

                        return;
                    }
                    else
                    {
                        CodedImgRGB = tmpCode;
                        ComptonImgRGB = tmpCompton;
                        HybridImgRGB = tmpHybrid;
                    }
                }
            }

            StatusUpdateMutex.ReleaseMutex();

        }


        private int timeInMiliSeconds = 0;
        public int TimeInMiliSeconds
        {
            get
            {
                return timeInMiliSeconds;
            }
            set
            {
                timeInMiliSeconds = value;
                OnPropertyChanged(nameof(TimeInMiliSeconds));

            }
        }

        private double s2M = 2;
        public double S2M
        {
            get
            {
                return s2M;
            }
            set
            {
                s2M = value;
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                OnPropertyChanged(nameof(S2M));
            }
        }

        private double m2D = 0.070;
        public double M2D
        {
            get
            {
                return m2D;
            }
            set
            {
                m2D = value;
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                OnPropertyChanged(nameof(M2D));
            }
        }

        private double minValuePortion = 0.70;
        public double MinValuePortion
        {
            get { return minValuePortion; }
            set
            {
                minValuePortion = value;
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);
                OnPropertyChanged(nameof(MinValuePortion));
            }
        }

        private double det_W = 0.312;
        public double Det_W
        {
            get { return det_W; }
            set { det_W = value; OnPropertyChanged(nameof(Det_W)); }
        }

        private double resImprov = 20;
        public double ResImprov
        {
            get
            {
                return resImprov;
            }
            set
            {
                resImprov = value;
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Status);

                OnPropertyChanged(nameof(ResImprov));
            }
        }

        private int imgSize = 800;
        public int ImgSize
        {
            get
            {
                return imgSize;
            }
            set
            {
                imgSize = value;
                OnPropertyChanged(nameof(ImgSize));
            }
        }

 

        private BitmapImage codedImgRGB;
        public BitmapImage CodedImgRGB
        {
            get { return codedImgRGB; }
            set
            {
                codedImgRGB = value;
                OnPropertyChanged(nameof(CodedImgRGB));
            }
        }

        private BitmapImage comptonImgRGB;
        public BitmapImage ComptonImgRGB
        {
            get { return comptonImgRGB; }
            set { comptonImgRGB = value; OnPropertyChanged(nameof(ComptonImgRGB)); }
        }


        private BitmapImage hybridImgRGB;
        public BitmapImage HybridImgRGB
        {
            get { return hybridImgRGB; }
            set { hybridImgRGB = value; OnPropertyChanged(nameof(HybridImgRGB)); }
        }


        private BitmapImage realtimeRGB;
        public BitmapImage RealtimeRGB
        {
            get { return realtimeRGB; }
            set
            {
                realtimeRGB = value;
                OnPropertyChanged(nameof(RealtimeRGB));
            }
        }


        private Task LoopTask;
        private bool RunLoop = true;
        private void Loop()
        {
            while (RunLoop)
            {
                BitmapImage? temp = LahgiApi.GetRgbImage();
                if (temp != null)
                {
                    RealtimeRGB = temp;
                }
                else
                {
                    Image tempImage = Image.FromFile("Resource/not_connected.jpg");

                    
                    using (MemoryStream ms = new MemoryStream())
                    {
                        tempImage.Save(ms, System.Drawing.Imaging.ImageFormat.Jpeg);

                        temp = new BitmapImage();
                        temp.BeginInit();
                        ms.Seek(0, SeekOrigin.Begin);
                        temp.StreamSource = ms;
                        temp.CacheOption = BitmapCacheOption.OnLoad;
                        temp.EndInit();
                        temp.Freeze();
                        //img = bitMapimg;
                    }
                    RealtimeRGB = temp;

                    Thread.Sleep(2000);

                }
            }
        }

        public override void Unhandle()
        {
            if (LoopTask != null)
            {
                RunLoop = false;
                LoopTask.Wait();
            }
        }
    }
}
