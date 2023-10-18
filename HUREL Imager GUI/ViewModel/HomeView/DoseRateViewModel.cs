using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using log4net;
using System.IO;

namespace HUREL_Imager_GUI.ViewModel
{
    //230920 sbkwon : DoseRate Alarm Level
    public enum enAlarm
    {
        enAlarmNone,
        enAlarm3,
        enAlarm4,
        enAlarm5,
    }

    //230920 sbkwon : DoseRate Alarm Event
    public class AlarmEventArgs : EventArgs
    {
        public enAlarm AlarmStatus { get; private set; }
        public AlarmEventArgs(enAlarm alarm)
        {
            AlarmStatus = alarm;
        }
    }

    public class DoseRateViewModel : ViewModelBase
    {
        public static EventHandler? AlarmUpdate;     //230920 sbkwon : DoseRate Alarm EventHandler

        //230920 sbkwon : DoseRate Alarm Invoke
        public static void AlarmUpdateInvoke(object? obj, enAlarm state)
        {
            Task.Run(() => { AlarmUpdate?.Invoke(obj, new AlarmEventArgs(state)); });
        }


        public DoseRateViewModel()
        {
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

                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading)
                {
                    var espect = LahgiApi.GetScatterSumSpectrumByTime(0);

                    (double dose, double std) = espect.GetAmbientDose(DoseCalcTime);
                    if (dose > 0.01)
                    {
                        DoseLogScale = Math.Log10(dose * 100);
                    }
                    else
                    {
                        DoseLogScale = 0;
                    }

                }
                if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                {
                    var espect = LahgiApi.GetScatterSumSpectrumByTime(DoseCalcTime);
                    (double dose, double std) = espect.GetAmbientDose(LahgiApi.ElapsedTime > DoseCalcTime ? DoseCalcTime : LahgiApi.ElapsedTime);    //231017 : sbkwon - if DoseCalcTime < 측정시간 then 측정시간 대입
                    if (dose > 0.01)
                    {
                        DoseLogScale = Math.Log10(dose * 100);
                    }
                    else
                    {
                        DoseLogScale = 0;
                    }

                }
            }
            StatusUpdateMutex.ReleaseMutex();
        }

        private double doseLogScale = Math.Log10(30 * 100);
        public double DoseLogScale
        {
            get { return doseLogScale; }
            set
            {

                //230921 sbkwon : doseRate Alarm 구분
                enAlarm enLevel = enAlarm.enAlarmNone;
                if (value >= 1.3 && value < 2.7) { enLevel = enAlarm.enAlarmNone; }
                else if (value >= 2.7 && value < 4.0) { enLevel = enAlarm.enAlarm4; }
                else if (value >= 4) { enLevel = enAlarm.enAlarm5; }

                if (oldAlarmLeve != enLevel)
                    AlarmUpdateInvoke(null, enLevel);

                oldAlarmLeve = enLevel;

                doseLogScale = value;
                OnPropertyChanged(nameof(DoseLogScale));
            }
        }

        private uint doseCalcTime = 5;
        public uint DoseCalcTime
        {
            get
            {
                return doseCalcTime;
            }
            set
            {
                doseCalcTime = value;
                OnPropertyChanged(nameof(DoseCalcTime));
            }
        }
        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= StatusUpdate;

            if (SoundPlayer != null)    //230921 sbkwon : 종료시
            {
                SoundPlayer.Stop();
                SoundPlayer.Dispose();
                SoundPlayer = null;
            }
        }

        //230911 sbkwon : 선량률에 따른 경보 설정 4~5단계
        private System.Media.SoundPlayer? SoundPlayer = null;
        private enAlarm oldAlarmLeve = enAlarm.enAlarmNone;

        //230911 sbkwon : 경보 단계 별 설정
        public void Alarm(enAlarm enLevel)
        {
            if (SoundPlayer != null)
            {
                SoundPlayer.Stop();
                SoundPlayer.Dispose();
                SoundPlayer = null;
            }

            if (enLevel < enAlarm.enAlarm3 && enLevel > enAlarm.enAlarm5)
                return;

            string? filepath = null;

            if (enLevel == enAlarm.enAlarm3)
            {
                filepath = "Sound\\Alarm3.wav";
            }
            else if (enLevel == enAlarm.enAlarm4)
            {
                filepath = "Sound\\Alarm4.wav";
            }
            else if (enLevel == enAlarm.enAlarm5)
            {
                filepath = "Sound\\Alarm5.wav";
            }

            if (!File.Exists(filepath))
            {
                LogManager.GetLogger(typeof(DoseRateViewModel)).Info($"not exist File : {filepath}");
                return;
            }

            SoundPlayer = new System.Media.SoundPlayer(filepath ?? string.Empty);

            SoundPlayer.PlayLooping();
        }
    }
}