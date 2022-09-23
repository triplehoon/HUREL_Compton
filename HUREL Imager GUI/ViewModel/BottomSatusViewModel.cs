using HUREL.Compton;
using log4net;
using log4net.Appender;
using log4net.Repository.Hierarchy;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    class BottomSatusViewModel : ViewModelBase
    {
        private string logMsg ="";
        public string LogMsg
        {
            get { return logMsg; }
            set { logMsg = value; OnPropertyChanged(nameof(LogMsg)); }
        }

        public BottomSatusViewModel()
        {
            LahgiApi.StatusUpdate += UpdateLogMsg;
            LahgiApi.StatusUpdate += StatusUpdate;
           Hierarchy? hierarchy = LogManager.GetRepository() as Hierarchy;
            mappender = hierarchy?.Root.GetAppender("MemoryAppender") as MemoryAppender;

        }

        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            if (!StatusUpdateMutex.WaitOne(100))
            {
                return;
            }
            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                {
                    var espect = LahgiApi.GetScatterSumSpectrumByTime(5);
                    (double dose, double std) = espect.GetAmbientDose(5);
                    DoseRateText = new string($"{dose:F3} ± {1.96 * std:F3} µSv/h");

                }
            }

            StatusUpdateMutex.ReleaseMutex();
        }

        private string doseRateText = new string($"{0.11615:F3} ± {1.96 * 0.123:F3} µSv/h");

        public string DoseRateText
        {
            get {  return doseRateText; }
            set { doseRateText = value;  OnPropertyChanged(nameof(DoseRateText)); }
        }

        private MemoryAppender? mappender;
        private void UpdateLogMsg(object? obj, EventArgs eventArgs)
        {
            var events = mappender?.PopAllEvents();
            
            if (events == null)
            {
                return;
            }
            foreach (var e in events)
            {
                if (e.MessageObject is string)
                {
                    LogMsg += e.Level + " " + e.TimeStamp.ToString("HH:mm::ss") + " [" + e.LoggerName + "]" + ": " + (string)e.MessageObject + "\n";
                }
                
            }
        }


        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= UpdateLogMsg;
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
