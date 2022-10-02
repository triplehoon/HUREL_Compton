using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Input;
using System.Windows.Media.Imaging;

namespace HUREL_Imager_GUI.ViewModel
{
    class InteractionPointViewModel : ViewModelBase
    {
        public InteractionPointViewModel()
        {
            var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
            var appSetting = configFile.AppSettings.Settings;
        
            if (appSetting[nameof(InteractionPointViewModel) + nameof(ImageSize)] == null)
            {
                appSetting.Add(nameof(InteractionPointViewModel) + nameof(ImageSize), "300");
                ImageSize = 300;
            }
            if (appSetting[nameof(InteractionPointViewModel) + nameof(PixelCount)] == null)
            {
                appSetting.Add(nameof(InteractionPointViewModel) + nameof(PixelCount), "500");
                PixelCount = 500;
            }
            if (appSetting[nameof(InteractionPointViewModel) + nameof(TimeInSecond)] == null)
            {
                appSetting.Add(nameof(InteractionPointViewModel) + nameof(TimeInSecond), "0");
                TimeInSecond = 0;
            }

            configFile.Save(ConfigurationSaveMode.Modified);
            ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);
            GenerateResponseImage().ConfigureAwait(false);

        }




        private BitmapImage scatterImg = new BitmapImage();
        public BitmapImage ScatterImg
        {
            get { return scatterImg; }
            set
            {
                scatterImg = value;
                OnPropertyChanged(nameof(ScatterImg));
            }
        }

        private BitmapImage absorberImg = new BitmapImage();
        public BitmapImage AbsorberImg
        {
            get { return absorberImg; }
            set
            {
                absorberImg = value;
                OnPropertyChanged(nameof(AbsorberImg));
            }
        }


        private AsyncCommand? _generateResponseImageCommand = null;
        public ICommand GenerateResponseImageCommand
        {
            get { return _generateResponseImageCommand ?? (_generateResponseImageCommand = new AsyncCommand(GenerateResponseImage)); }
        }

        private int imageSize = Convert.ToInt32(ConfigurationManager.AppSettings.Get(nameof(InteractionPointViewModel)+nameof(ImageSize)));
        public int ImageSize
        {
            get { return imageSize; }
            set { imageSize = value;

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(InteractionPointViewModel) + nameof(ImageSize)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                OnPropertyChanged(nameof(ImageSize)); }
        }

        private int pixelCount = Convert.ToInt32(ConfigurationManager.AppSettings.Get(nameof(InteractionPointViewModel) + nameof(PixelCount)));
        public int PixelCount
        {
            get { return pixelCount; }
            set { pixelCount = value;

                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(InteractionPointViewModel) + nameof(PixelCount)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);

                OnPropertyChanged(nameof(PixelCount)); }
        }

        private double timeInSecond = Convert.ToDouble(ConfigurationManager.AppSettings.Get(nameof(InteractionPointViewModel) + nameof(TimeInSecond)));
        public double TimeInSecond
        {
            get { return timeInSecond; }
            set { timeInSecond = value;
                var configFile = ConfigurationManager.OpenExeConfiguration(ConfigurationUserLevel.None);
                var appSetting = configFile.AppSettings.Settings;
                appSetting[nameof(InteractionPointViewModel) + nameof(TimeInSecond)].Value = value.ToString();

                configFile.Save(ConfigurationSaveMode.Modified);
                ConfigurationManager.RefreshSection(configFile.AppSettings.SectionInformation.Name);


                OnPropertyChanged(nameof(TimeInSecond)); }
        }

        private async Task GenerateResponseImage()
        {
            await Task.Run(()=>
                {
                    BitmapImage? temp = null;
                    temp = LahgiApi.GetResponseImage(ImageSize, PixelCount, TimeInSecond, true);
                    if (temp != null)
                    {
                        ScatterImg = temp;
                    }
                    temp = LahgiApi.GetResponseImage(ImageSize, PixelCount, TimeInSecond, false);
                    if (temp != null)
                    {
                        AbsorberImg = temp;
                    }
                });
        }
        public override void Unhandle()
        {
            
        }
    }
}
