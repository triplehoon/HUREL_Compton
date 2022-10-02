using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL_Imager_GUI.ViewModel
{
    public class angle
    {
        public double Direction { get; set; }
        public double Value { get; set; }
        public angle(double direction, double value)
        {
            Direction = direction;
            Value = value;
        }
    }
    
    public class SourceDirectionViewModel : ViewModelBase
    {
        
        public SourceDirectionViewModel()
        {
            TestAngle = new ObservableCollection<angle>();
            Random random = new Random();
            TestAngle.Add(new angle(0, 50));
            for (int i = 20; i < 359; i += 20)
            {
                TestAngle.Add(new angle(i, random.NextDouble() * 20 + 5));
            }
        }
        public ObservableCollection<angle> TestAngle
        {
            get; set;
        }

        public override void Unhandle()
        {
            
        }
    }
}
