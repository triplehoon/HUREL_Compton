using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using HUREL_Imager_GUI.Components;
using Syncfusion.CompoundFile.XlsIO.Native;
using Syncfusion.UI.Xaml.Charts;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Configuration;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using static HelixToolkit.Wpf.SharpDX.Model.Metadata;

namespace HUREL_Imager_GUI.ViewModel
{
    public class IsotopeInfo
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public string Energy { get; set; }
        // public string Dose { get; set; }
        public IsotopeInfo(string name, string description, string energy, string dose)
        {
            Name = name;
            Description = description;
            Energy = energy;
            //Dose = dose;
        }
    }
    public enum eSpectrumCases
    {
        Scatter,
        Absorber,
        All,
        ByChannel
    };

    //#nullable disable

    //231016 sbkwon : 소스 객체 찾기
    public class SpectrumViewModel : ViewModelBase
    {
        /// <summary>소스 객체 찾기</summary>
        /// <remarks>traceLog를 true로 하면, System.Diagnostics.Trace로만 경로를 남김</remarks>
        /// <typeparam name="T">확인 객체형</typeparam>
        /// <param name="routedEventArgs">추적할 이벤트</param>
        /// <param name="traceLog">트리 추적 로그</param>
        /// <returns>소스 객체 검색 결과 반환</returns>
        public static (bool isExist, T resultObject) FindSourceObject<T>(RoutedEventArgs routedEventArgs)
            where T : DependencyObject
        {
            var hit = routedEventArgs.OriginalSource as DependencyObject;
            var source = routedEventArgs.Source as DependencyObject;

            if (source is T) return (true, source as T);

            while (null != hit && source != hit)
            {
                //Trace.WriteLine(hit);

                if (hit is T result) return (true, result);

                try { hit = VisualTreeHelper.GetParent(hit); }
                catch { break; }
            }

            return (false, default(T));
        }

        public SpectrumViewModel()
        {
            //231016 sbkwon : S 마우스 이벤트
            MouseEnterCommand = new AsyncCommand<MouseEventArgs>(OnMouseEnter);
            MouseLeaveCommand = new AsyncCommand<MouseEventArgs>(OnMouseLeave);
            PreviewMouseDownCommand = new AsyncCommand<MouseButtonEventArgs>(OnPreviewMouseDown);
            PreviewMouseUpCommand = new AsyncCommand<MouseButtonEventArgs>(OnPreviewMouseUp);
            PreviewMouseMoveCommand = new AsyncCommand<MouseEventArgs>(OnPreviewMouseMove);
            PreviewMouseDoubleClickCommand = new AsyncCommand<MouseButtonEventArgs>(OnPreviewMouseDoubleClick);
            //231016 sbkwon : E 마우스 이벤트

            LahgiApi.StatusUpdate += StatusUpdate;
            EnergySpectrum = new ObservableCollection<HistoEnergy>();
            IsotopeInfos = new ObservableCollection<IsotopeInfo>();
            Random random = new Random();
            for (int i = 0; i < 3000; i++)
            {
                EnergySpectrum.Add(new HistoEnergy(i, random.Next(5000)));
            }

            //test sbkwon
            //PeakLine = new ObservableCollection<GraphData>
            //{
            //    new GraphData(500, 1000 * 1.5),
            //    new GraphData(800, 1000 * 1.5),
            //    new GraphData(1000, 1000 * 1.5),
            //    new GraphData(1200, 1000 * 1.5),
            //    new GraphData(1500, 1000 * 1.5),
            //    new GraphData(2000, 1000 * 1.5)
            //};

            //IsotopeInfos = new ObservableCollection<IsotopeInfo>
            //{
            //   new IsotopeInfo("Co-58", "가동중 원전", "810.76, 511", ""),
            //   new IsotopeInfo("Co-60", "가동중 원전/비파괴 검사", "1332.49, 1173.23", "")
            //};

            //List<Isotope> DetectedIso = new List<Isotope>
            //{
            //    PeakSearching.IsotopeList.ElementAt(3),
            //    PeakSearching.IsotopeList.ElementAt(8),
            //    PeakSearching.IsotopeList.ElementAt(17),
            //    PeakSearching.IsotopeList.ElementAt(1),
            //};

            //if (DetectedIso.Count > 0 && SelectPeakLine.Count == 0)
            //{
            //    double selectE = 0;
            //    int priority = 9999;
            //    foreach (var item in DetectedIso)
            //    {
            //        foreach (var e in item.Priority.Select((value, index) => (value, index)))
            //        {
            //            if (priority > e.value)
            //            {
            //                priority = e.value;
            //                selectE = item.PeakEnergy[e.index];
            //            }
            //        }
            //    }

            //    if (selectE != 0) AddSelectPeak(selectE);
            //}
            //test sbkwon
        }
        Mutex StatusUpdateMutex = new Mutex();
        static int IsotopeInfosCount = 0;

        //231016 sbkwon : S 마우스 이벤트
        public ICommand MouseEnterCommand { get; }
        public ICommand MouseLeaveCommand { get; }
        public ICommand PreviewMouseDownCommand { get; }
        public ICommand PreviewMouseUpCommand { get; }
        public ICommand PreviewMouseMoveCommand { get; }
        public ICommand PreviewMouseDoubleClickCommand { get; }

        /// <summary>마우스 겹쳐짐 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnMouseEnter(MouseEventArgs e) { }

        /// <summary>마우스 벗어남 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnMouseLeave(MouseEventArgs e) { }

        /// <summary>마우스 버튼 눌림 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseDown(MouseButtonEventArgs e)
        {
            var (isExists, obj) = FindSourceObject<Syncfusion.UI.Xaml.Charts.ChartRootPanel>(e);

            if (isExists is true)
            {
                Point p = e.GetPosition((IInputElement)obj);
                int X = (int)p.X;

                if ((X - MouseXPointZero) > 0 && PeakLine.Count() > 0)
                {
                    double xRatio = 3000.0 / (obj.ActualWidth - MouseXPointZero);   //좌표 변환 상수
                    int nHitEnergy = (int)((X - MouseXPointZero) * xRatio);           //에너지 값

                    //PeakLine hit 판단
                    foreach (var peak in PeakLine)
                    {
                        if (Math.Abs(peak.X - nHitEnergy) <= HitRange)
                        {
                            //핵종 선택에 대한 energy gate 설정
                            AddSelectPeak(peak.X);
                            //System.Diagnostics.Trace.WriteLine($" hit : {peak.X}, Ratio : {PeakLine.Count()} W : {obj.ActualWidth}, H : {obj.ActualHeight}");
                        }
                    }
                }
            }
        }

        /// <summary>마우스 버튼 올라옴 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseUp(MouseButtonEventArgs e) { }

        /// <summary>마우스 움직임 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseMove(MouseEventArgs e) { }

        /// <summary>마우스 버튼 두번 누름 터널링 이벤트 처리</summary>
        /// <param name="e"></param>
        private async Task OnPreviewMouseDoubleClick(MouseButtonEventArgs e) { }
        //231016 sbkwon : E

        //231016 sbkwon : 선택한 Peak Line 리스트
        private ObservableCollection<double> _selectPeakLine = new ObservableCollection<double>(); //Spectrum 에서 Peak Line 선택 리스트
        public ObservableCollection<double> SelectPeakLine
        {
            get { return _selectPeakLine; }
            set { _selectPeakLine = value; OnPropertyChanged(nameof(SelectPeakLine)); }
        }

        //231017 sbkwon : 선택한 Peak Line 추가
        public void AddSelectPeak(double energe)
        {
            bool bExist = false;    //기존에 선택여부 확인

            foreach (var peak in SelectPeakLine)
            {
                if (peak == energe)
                {
                    bExist = true; break;
                }
            }

            if (bExist)
                SelectPeakLine.Remove(energe);  //기존 선택된것은 삭제
            else
                SelectPeakLine.Add(energe);     //기존 선택없으면 추가

            //선택된 Peak를 영상화 하기 위해 Echk 계산 (Min , Max)
            var tempEchk = new List<AddListModeDataEchk>();
            foreach (var Echk in SelectPeakLine)
            {
                double fwhm = PeakSearching.CalcFWHM(Echk, Ref_x, Ref_fwhm, Ref_at_0);
                double MinE = Echk - fwhm;
                double MaxE = Echk + fwhm;


                AddListModeDataEchk addListModeDataEchk = new AddListModeDataEchk(MinE, MaxE);
                tempEchk.Add(addListModeDataEchk);

                //Trace.WriteLine($"{Echk}, {MinE}, {MaxE}");
            }

            //Trace.WriteLine("----------------------");

            //lahgi Echks에 추가
            LahgiApi.Echks = tempEchk;
        }

        //231016 sbkwon : Spectrum X=0 좌표 마우스 포인트 값 설정
        private int _mouseXPointZero = 55;
        public int MouseXPointZero
        {
            get => _mouseXPointZero;
            set => _mouseXPointZero = value;
        }

        //231016 sbkwon : Hit Peak X Range (Peak value를 기준으로 일정 범위 내에 hit로 간주할 값)
        private int _hitRange = 10;
        public int HitRange
        {
            get => _hitRange; set => _hitRange = value;
        }

        public void StatusUpdate(object? obj, EventArgs eventArgs)
        {
            if (!StatusUpdateMutex.WaitOne(0))
            {
                return;
            }

            SpectrumEnergyNasa? spectrum = null;
            switch (_spectrumCases)
            {
                case eSpectrumCases.Scatter:
                    spectrum = LahgiApi.GetScatterSumSpectrumByTime(10);
                    break;
                case eSpectrumCases.Absorber:
                    spectrum = LahgiApi.GetAbsorberSumSpectrum();
                    break;
                case eSpectrumCases.All:
                    spectrum = LahgiApi.GetSumSpectrumEnergy();
                    break;
                case eSpectrumCases.ByChannel:
                    spectrum = LahgiApi.GetSpectrumEnergy(FpgaChannelNumber);
                    break;
            }
            if (spectrum != null)
            {
                EnergySpectrum = new ObservableCollection<HistoEnergy>(spectrum.HistoEnergies);
                int maxCount = 0;
                for (int i = 0; i < spectrum.HistoEnergies.Count; i++)
                {
                    if (spectrum.HistoEnergies[i].Count > maxCount)
                    {
                        maxCount = spectrum.HistoEnergies[i].Count;
                    }
                }
                if (eventArgs is LahgiApiEnvetArgs)
                {
                    LahgiApiEnvetArgs lahgiApiEnvetArgs = (LahgiApiEnvetArgs)eventArgs;

                    if (lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Loading || lahgiApiEnvetArgs.State == eLahgiApiEnvetArgsState.Spectrum)
                    {
                        var espect = spectrum;

                        ObservableCollection<IsotopeInfo> isotopeInfos = new ObservableCollection<IsotopeInfo>();
                        List<Isotope> DetectedIso = PeakSearching.GetIsotopesFromPeaks(espect.FindPeaks(Ref_x, Ref_fwhm, Ref_at_0, Min_snr), 1, Ref_x, Ref_fwhm, Ref_at_0);


                        if (IsSpectrumAnalysisShow)
                        {

                            List<GraphData> graphDatas = new List<GraphData>();
                            for (int i = 0; i < espect.SnrData.Count; i++)
                            {
                                GraphData graphData = espect.SnrData[i];
                                graphDatas.Add(graphData);
                            }
                            SnrSpectrum = new ObservableCollection<GraphData>(graphDatas);

                            List<GraphData> graphDatas2 = new List<GraphData>();

                            for (int i = 0; i < espect.SnrData.Count; i++)
                            {
                                GraphData graphData = new GraphData(espect.SnrData[i].X, espect.SnrData[i].Y);
                                graphData.Y = Min_snr;
                                graphDatas2.Add(graphData);
                            }
                            MinSnrLine = new ObservableCollection<GraphData>(graphDatas2);

                            PeakLine = new ObservableCollection<GraphData>(espect.PeakData);
                        }
                        else
                        {
                            SnrSpectrum = new ObservableCollection<GraphData>();
                            MinSnrLine = new ObservableCollection<GraphData>();
                            PeakLine = new ObservableCollection<GraphData>();
                        }
                        PeakLine = new ObservableCollection<GraphData>();
                        bool isBa133Found = false;
                        bool isCo60Found = false;
                        foreach (Isotope iso in DetectedIso)
                        {
                            string energy = "";
                            foreach (double e in iso.PeakEnergy)
                            {
                                energy += e.ToString("0.");
                                energy += " ";
                            }



                            isotopeInfos.Add(new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, energy, ""));
                            foreach (double e in iso.PeakEnergy)
                            {
                                GraphData peakData = new GraphData(e, maxCount * 1.5);
                                PeakLine.Add(peakData);
                            }

                            //231017 sbkwon : sub Peak Energy => Peak Line 추가
                            foreach (double e in iso.SubPeakEnergy)
                            {
                                double fwhm = PeakSearching.CalcFWHM(e, Ref_x, Ref_fwhm, Ref_at_0);
                                foreach (var data in espect.PeakData)
                                {
                                    if (e - fwhm < data.X && e + fwhm > data.X)
                                    {
                                        GraphData peakData = new GraphData(e, maxCount * 1.5);
                                        PeakLine.Add(peakData);
                                    }
                                }
                            }

                            if (iso.IsotopeElement == IsotopeElement.Co60)
                            {
                                isCo60Found = true;
                            }
                            if (iso.IsotopeElement == IsotopeElement.Ba133)
                            {
                                isBa133Found = true;
                            }

                        }

                        IsotopeInfosCount = isotopeInfos.Count;
                        IsotopeInfos = isotopeInfos;

                        //231017 sbkwon : 우선순위
                        if (DetectedIso.Count > 0 && SelectPeakLine.Count == 0)
                        {
                            double selectE = 0;
                            int priority = 9999;
                            foreach (var item in DetectedIso)
                            {
                                foreach (var e in item.Priority.Select((value, index) => (value, index)))
                                {
                                    if (priority > e.value)
                                    {
                                        priority = e.value;
                                        selectE = item.PeakEnergy[e.index];
                                    }
                                }
                            }

                            if (selectE != 0) AddSelectPeak(selectE);
                        }

                        //SpectrumEnergyNasa copySpect = new SpectrumEnergyNasa(spectrum);
                        /*
                        if (isBa133Found)
                        {
                            for (int i =0; i < copySpect.HistoEnergies.Count; i++)
                            {
                                if ((copySpect.HistoEnergies[i].Energy < 400 && copySpect.HistoEnergies[i].Energy > 300))
                                {

                                }
                                else
                                {
                                    copySpect.HistoEnergies[i].Count = 0;
                                }
                            }
                            ImagingEnergySpectrum = new ObservableCollection<HistoEnergy>(copySpect.HistoEnergies);


                            var tempEchk = new List<AddListModeDataEchk>();
                            //tempEchk.Add(new AddListModeDataEchk(30, 90));
                            //tempEchk.Add(new AddListModeDataEchk(60, 100));
                            //tempEchk.Add(new AddListModeDataEchk(330, 370));
                            //tempEchk.Add(new AddListModeDataEchk(450, 570));
                            //tempEchk.Add(new AddListModeDataEchk(1200, 1350));
                            //tempEchk.Add(new AddListModeDataEchk(60, 100));

                            tempEchk.Add(new AddListModeDataEchk(330, 370));
                            //tempEchk.Add(new AddListModeDataEchk(1173 - 70, 1173 + 70));
                            //tempEchk.Add(new AddListModeDataEchk(1333 - 50, 1333 + 50));
                            LahgiApi.Echks = tempEchk;
                        }
                        else
                        {
                            if (isCo60Found)
                            {

                                for (int i = 0; i < copySpect.HistoEnergies.Count; i++)
                                {


                                    if ((copySpect.HistoEnergies[i].Energy < 1173 + 70 && copySpect.HistoEnergies[i].Energy > 1173 - 70) || (copySpect.HistoEnergies[i].Energy < 1333 + 50 && copySpect.HistoEnergies[i].Energy > 1333 - 50))
                                    {

                                    }
                                    else
                                    {
                                        copySpect.HistoEnergies[i].Count = 0;
                                    }
                                }
                                ImagingEnergySpectrum = new ObservableCollection<HistoEnergy>(copySpect.HistoEnergies);



                                var tempEchk = new List<AddListModeDataEchk>();
                                //tempEchk.Add(new AddListModeDataEchk(30, 90));
                                //tempEchk.Add(new AddListModeDataEchk(60, 100));
                                //tempEchk.Add(new AddListModeDataEchk(330, 370));
                                //tempEchk.Add(new AddListModeDataEchk(450, 570));
                                //tempEchk.Add(new AddListModeDataEchk(1200, 1350));
                                //tempEchk.Add(new AddListModeDataEchk(60, 100));

                                //tempEchk.Add(new AddListModeDataEchk(330, 370));
                                tempEchk.Add(new AddListModeDataEchk(1173 - 70, 1173 + 70));
                                tempEchk.Add(new AddListModeDataEchk(1333 - 50, 1333 + 50));
                                LahgiApi.Echks = tempEchk;
                            }
                            else
                            {
                                ImagingEnergySpectrum = new ObservableCollection<HistoEnergy>();
                            }


                        }
                        */
                    }


                }

            }

            StatusUpdateMutex.ReleaseMutex();
        }
        private ObservableCollection<HistoEnergy> _energySpetrum = new ObservableCollection<HistoEnergy>();

        public ObservableCollection<HistoEnergy> EnergySpectrum
        {
            get { return _energySpetrum; }
            set
            {
                _energySpetrum = value;
                OnPropertyChanged(nameof(EnergySpectrum));
            }
        }

        private ObservableCollection<HistoEnergy> _imagingEnergySpetrum = new ObservableCollection<HistoEnergy>();

        public ObservableCollection<HistoEnergy> ImagingEnergySpectrum
        {
            get { return _imagingEnergySpetrum; }
            set
            {
                _imagingEnergySpetrum = value;
                OnPropertyChanged(nameof(ImagingEnergySpectrum));
            }
        }
        public ObservableCollection<IsotopeInfo> _isotopeInfos = new ObservableCollection<IsotopeInfo>();
        public ObservableCollection<IsotopeInfo> IsotopeInfos
        {
            get
            {
                return _isotopeInfos;
            }
            set
            {
                if (_isotopeInfos.Count != value.Count)
                {
                    _isotopeInfos = value;
                    OnPropertyChanged(nameof(IsotopeInfos));
                }
                else
                {
                    for (int i = 0; i < IsotopeInfos.Count; i++)
                    {
                        if (value[i].Name != IsotopeInfos[i].Name)
                        {
                            _isotopeInfos = value;
                            OnPropertyChanged(nameof(IsotopeInfos));
                        }
                    }
                }

            }
        }
        private ObservableCollection<GraphData> _minSnrLine = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> MinSnrLine
        {
            get { return _minSnrLine; }
            set
            {
                _minSnrLine = value;
                OnPropertyChanged(nameof(MinSnrLine));
            }
        }

        private ObservableCollection<GraphData> _snrSpectrum = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> SnrSpectrum
        {
            get { return _snrSpectrum; }
            set
            {
                _snrSpectrum = value;
                OnPropertyChanged(nameof(SnrSpectrum));
            }
        }

        private ObservableCollection<GraphData> _peakLine = new ObservableCollection<GraphData>();

        public ObservableCollection<GraphData> PeakLine
        {
            get { return _peakLine; }
            set
            {
                _peakLine = value;
                OnPropertyChanged(nameof(PeakLine));
            }
        }

        private eSpectrumCases _spectrumCases = eSpectrumCases.Scatter;
        public eSpectrumCases SpectrumCases
        {
            get { return _spectrumCases; }
            set
            {
                _spectrumCases = value;
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
                OnPropertyChanged(nameof(SpectrumCases));
            }
        }
        private int fpgaChannelNumber = 0;
        public int FpgaChannelNumber
        {
            get { return fpgaChannelNumber; }
            set
            {
                if (fpgaChannelNumber >= 0 && fpgaChannelNumber <= 15)
                {
                    fpgaChannelNumber = value;
                }
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
                OnPropertyChanged(nameof(FpgaChannelNumber));
            }
        }

        public float Ref_x
        {
            get { return LahgiApi.Ref_x; }
            set
            {
                LahgiApi.Ref_x = value;
                OnPropertyChanged(nameof(Ref_x));
            }
        }

        public float Ref_fwhm
        {
            get { return LahgiApi.Ref_fwhm; }
            set
            {
                LahgiApi.Ref_fwhm = value;
                OnPropertyChanged(nameof(Ref_fwhm));
            }
        }
        public float Ref_at_0
        {
            get { return LahgiApi.Ref_at_0; }
            set
            {
                LahgiApi.Ref_at_0 = value;

                OnPropertyChanged(nameof(Ref_at_0));
            }
        }
        public float Min_snr
        {
            get { return LahgiApi.Min_snr; }
            set
            {
                LahgiApi.Min_snr = value;
                OnPropertyChanged(nameof(Min_snr));
            }
        }

        private bool isSpectrumAnalysisShow = false;
        public bool IsSpectrumAnalysisShow
        {
            get { return isSpectrumAnalysisShow; }
            set
            {
                isSpectrumAnalysisShow = value;
                LahgiApi.StatusUpdateInvoke(null, eLahgiApiEnvetArgsState.Spectrum);
                OnPropertyChanged(nameof(IsSpectrumAnalysisShow));
            }
        }

        public override void Unhandle()
        {
            LahgiApi.StatusUpdate -= StatusUpdate;
        }
    }
}
