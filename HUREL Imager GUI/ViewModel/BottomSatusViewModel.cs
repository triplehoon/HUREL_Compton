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
            LahgiApi.StatusUpdate += StatusUpdate;
           Hierarchy? hierarchy = LogManager.GetRepository() as Hierarchy;
            mappender = hierarchy?.Root.GetAppender("MemoryAppender") as MemoryAppender;

        }

        Mutex StatusUpdateMutex = new Mutex();
        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {

            if (eventArgs is LahgiApiEnvetArgs)
            {
                LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;
                ElapsedTime = LahgiApi.SessionStopwatch.Elapsed.ToString(@"mm\:ss");
                if (!StatusUpdateMutex.WaitOne(0))
                {
                    return;
                }
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                {
                    var espect = LahgiApi.GetScatterSumSpectrumByTime(5);
                    (double dose, double std) = espect.GetAmbientDose(5);
                    DoseRateText = new string($"{dose:F3} ± {1.96 * std:F3} µSv/h");

                }
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Status)
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
                            logMsg += e.Level + " " + e.TimeStamp.ToString("HH:mm::ss") + " [" + e.LoggerName + "]" + ": " + (string)e.MessageObject + "\n";
                        }

                    }
                    OnPropertyChanged(nameof(LogMsg));
                }
            }

            StatusUpdateMutex.ReleaseMutex();
        }
        private string elapsedTime = LahgiApi.SessionStopwatch.Elapsed.ToString(@"mm\:ss");
        public string ElapsedTime
        {
            get
            {
                return elapsedTime;
            }
            set
            {
                elapsedTime = value;
                OnPropertyChanged(nameof(ElapsedTime));
            }
        }

        private string doseRateText = new string($"{0.11615:F3} ± {1.96 * 0.123:F3} µSv/h");

        public string DoseRateText
        {
            get {  return doseRateText; }
            set { doseRateText = value;  OnPropertyChanged(nameof(DoseRateText)); }
        }

        private MemoryAppender? mappender;
     


        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
