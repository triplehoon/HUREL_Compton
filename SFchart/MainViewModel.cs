using GalaSoft.MvvmLight.Command;
using HUREL.Compton;
using HUREL.Compton.LACC;
using MathNet.Numerics.Statistics;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace SFchart
{
    public class MainViewModel : INotifyPropertyChanged
    {
        public LACC_Control LACC_Control { get; private set; }

        public MainViewModel()
        {
            Positions = new ObservableCollection<Position>();
            Task.Run(() => Loading());
        }

        public bool HistogramUpdate { get; set; }

        public string Status { get; set; }
        public string Counts { get; set; }

        public void Loading()
        {
            #region Random Spect
            Random rand = new Random();
            List<double> randDobuleList = new List<double>();
            
            for (int i = 0; i < 10000; i++)
            {
                int num = rand.Next(0, 1000);
                randDobuleList.Add(num);
            }

            SpectrumHisto spectrum = new SpectrumHisto(randDobuleList, 200, 0, 1000);

            SpectrumHistoModels = spectrum.SpectrumData;
            #endregion


            for (int i = 0; i < 50000; i++)
            {
                Positions.Add(new Position(rand.NextDouble()*200,rand.NextDouble()*200));
            }
        }

        public record Position(double X, double Y);
        public ObservableCollection<Position> Positions { get; }

        public BlockingCollection<short[]> ShortBuffer = new BlockingCollection<short[]>();
    


       

        private ObservableCollection<SpectrumHisto.SpectrumHistoModel> spectrumHistoModels;
        public ObservableCollection<SpectrumHisto.SpectrumHistoModel> SpectrumHistoModels
        {
            get { return spectrumHistoModels; }
            set { spectrumHistoModels = value; OnPropertyChanged(nameof(SpectrumHistoModels)); }
        }
      
        public class SpectrumHisto
        {
            public ObservableCollection<SpectrumHistoModel> SpectrumData = new ObservableCollection<SpectrumHistoModel>();
            public SpectrumHisto(IEnumerable<double> data, int nbuckets, double lower, double upper)
            {
                Histogram hist = new Histogram(data, nbuckets, lower, upper);                 
                for(int i = 0; i < hist.BucketCount;i++)
                {
                    SpectrumData.Add(new SpectrumHistoModel { LowerBound = hist[i].LowerBound, Count = hist[i].Count });
                }
                
            }

            public class SpectrumHistoModel
            {
                public double LowerBound { get; set; }
                public double Count { get; set; }
            }
        }

        public Task ReadLine;
        public Task AddLACC;

        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
        public event PropertyChangedEventHandler PropertyChanged;
      
    }


}
