﻿using HUREL.Compton;
using HUREL.Compton.RadioisotopeAnalysis;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Configuration;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    public class IsotopeInfo
    {
        public string Name { get; set; }
        public string Description { get; set; }
        public string Energy { get; set; }
        public string Dose { get; set; }
        public IsotopeInfo(string name, string description, string energy, string dose)
        {
            Name = name;
            Description = description;
            Energy = energy;
            Dose = dose;
        }
    }
    public enum eSpectrumCases
    {
        Scatter,
        Absorber,
        All,
        ByChannel
    };

    public class SpectrumViewModel : ViewModelBase
    {

        public SpectrumViewModel()
        {
            LahgiApi.StatusUpdate += StatusUpdate;
            EnergySpectrum = new ObservableCollection<HistoEnergy>();
            IsotopeInfos = new ObservableCollection<IsotopeInfo>();
            Random random = new Random();
            for (int i = 0; i < 3000; i++)
            {
                EnergySpectrum.Add(new HistoEnergy(i, random.Next(5000)));
            }
        }
        Mutex StatusUpdateMutex = new Mutex();
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
                    spectrum = LahgiApi.GetScatterSumSpectrum();
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

                        foreach (Isotope iso in DetectedIso)
                        {
                            string energy = "";
                            foreach (double e in iso.PeakEnergy)
                            {
                                energy += e.ToString("0.");
                                energy += " ";
                            }

                            isotopeInfos.Add(new IsotopeInfo(iso.IsotopeName, iso.IsotopeDescription, energy, ""));
                        }
                        IsotopeInfos = isotopeInfos;

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
        public ObservableCollection<IsotopeInfo> _isotopeInfos = new ObservableCollection<IsotopeInfo>();
        public ObservableCollection<IsotopeInfo> IsotopeInfos
        {
            get
            {
                return _isotopeInfos;
            }
            set
            {
                _isotopeInfos = value;
                OnPropertyChanged(nameof(IsotopeInfos));
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