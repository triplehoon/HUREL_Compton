using Compton_GUI_WPF.ViewModel;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Windows.Media;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Data;
using static Compton_GUI_WPF.ViewModel.MainViewModel;

namespace Compton_GUI_WPF.View
{


    public class EnumToBooleanConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;
            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            if (Enum.IsDefined(value.GetType(), value) == false)
            {
                return DependencyProperty.UnsetValue;
            }

            object parameterValue = Enum.Parse(value.GetType(), parameterString);

            return parameterValue.Equals(value);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;

            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            return Enum.Parse(targetType, parameterString);
        }
    }

    public class EReconProjectionToRangeConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;
            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            if (Enum.IsDefined(value.GetType(), value) == false)
            {
                return DependencyProperty.UnsetValue;
            }
            EReconProjection projection = (EReconProjection)value;
            
            if (parameterString == "minusX")
            {
                switch (projection)
                {
                    case EReconProjection.XY:
                        return -0.50;
                    case EReconProjection.XZ:
                        return -0.50;
                    case EReconProjection.ZY:
                        return 0;
                }
            }
            if (parameterString == "plusX")
            {
                switch (projection)
                {
                    case EReconProjection.XY:
                        return 0.50;
                    case EReconProjection.XZ:
                        return 0.50;
                    case EReconProjection.ZY:
                        return 1.00;
                }
            }
            if (parameterString == "minusY")
            {
                switch (projection)
                {
                    case EReconProjection.XY:
                        return -0.50;
                    case EReconProjection.XZ:
                        return 0;
                    case EReconProjection.ZY:
                        return -0.50;
                }
            }
            if (parameterString == "plusY")
            {
                switch (projection)
                {
                    case EReconProjection.XY:
                        return 0.50;
                    case EReconProjection.XZ:
                        return 1.00;
                    case EReconProjection.ZY:
                        return 0.50;
                }
            }

            return new NotImplementedException();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            throw new NotImplementedException();
        }
    }


    public class EReconProjectionToAxisNameConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;
            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            if (Enum.IsDefined(value.GetType(), value) == false)
            {
                return DependencyProperty.UnsetValue;
            }
            EReconProjection projection = (EReconProjection)value;

            if (parameterString == "X")
            {
                switch (projection)
                {
                    case EReconProjection.XY:
                        return "X [m]";
                    case EReconProjection.XZ:
                        return "X [m]";
                    case EReconProjection.ZY:
                        return "Z [m]";
                }
            }
            if (parameterString == "Y")
            {
                switch (projection)
                {
                    case EReconProjection.XY:
                        return "Y [m]";
                    case EReconProjection.XZ:
                        return "Z [m]";
                    case EReconProjection.ZY:
                        return "Y [m]";
                }
            }        

            return new NotImplementedException();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            throw new NotImplementedException();
        }
    }

    public class EReconProjectionToInverseXConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            string parameterString = parameter as string;
            if (parameterString == null)
            {
                return DependencyProperty.UnsetValue;
            }

            if (Enum.IsDefined(value.GetType(), value) == false)
            {
                return DependencyProperty.UnsetValue;
            }
            EReconProjection projection = (EReconProjection)value;


            switch (projection)
            {
                case EReconProjection.XY:
                    return true;
                case EReconProjection.XZ:
                    return true;
                case EReconProjection.ZY:
                    return false;
            }


            return new NotImplementedException();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo cultureInfo)
        {
            throw new NotImplementedException();
        }
    }

    public class TimespaneToStringConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            var timeSpan = (TimeSpan)value;
            return timeSpan.ToString();
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            TimeSpan timeSpan = TimeSpan.FromSeconds(System.Convert.ToDouble((string)value));
            return timeSpan;
        }
    }

    public class ObservalbeDoubleToChannelNameConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value == null)
            {
                List<string> non = new List<string>();
                non.Add("PMT CorrMat");
                return non;
            }
            var channelGain = (ObservableCollection<double>)value;
            ObservableCollection<string> CorrMatNameAndGain = new ObservableCollection<string>();
            int i = 0;
            CorrMatNameAndGain.Add("PMT CorrMat");
            foreach (double d in channelGain)
            {
                CorrMatNameAndGain.Add("CorrMat[" + i + "]: " + d.ToString("F2"));
                i++;
            }
            return CorrMatNameAndGain;
        }
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }

    [ValueConversion(typeof(bool), typeof(bool))]
    public class InverseBooleanConverter : IValueConverter
    {
        #region IValueConverter Members

        public object Convert(object value, Type targetType, object parameter,
            System.Globalization.CultureInfo culture)
        {
            if (targetType != typeof(bool))
                throw new InvalidOperationException("The target must be a boolean");

            return !(bool)value;
        }

        public object ConvertBack(object value, Type targetType, object parameter,
            System.Globalization.CultureInfo culture)
        {
            throw new NotSupportedException();
        }

        #endregion
    }

    public class BoolToBrushConverter : BoolToValueConverter<String> { }
    public class BoolToStringConverter : BoolToValueConverter<String> { }
    public class BoolToVisibilityConverter : BoolToValueConverter<Visibility> { }
    public class BoolToValueConverter<T> : IValueConverter
    {
        public T FalseValue { get; set; }
        public T TrueValue { get; set; }

        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value == null)
                return FalseValue;
            else
                return (bool)value ? TrueValue : FalseValue;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return value != null ? value.Equals(TrueValue) : false;
        }
    }


}
