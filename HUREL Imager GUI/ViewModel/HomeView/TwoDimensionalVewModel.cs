using HUREL.Compton;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;

namespace HUREL_Imager_GUI.ViewModel
{
    internal class TwoDimensionalVewModel : ViewModelBase
    {

        public TwoDimensionalVewModel()
        {
            LoopTask = Task.Run(Loop);

            realtimeRGB = new BitmapImage();

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

            }
            StatusUpdateMutex.ReleaseMutex();

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
            }
        }


        public override void Unhandle()
        {
            
        }
    }
}
