using System.Configuration;
using System.Diagnostics;
using System.IO;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using HUREL.Compton.RadioisotopeAnalysis;
using log4net;
using Newtonsoft.Json.Linq;


namespace HUREL.Compton
{


    public class LahgiSession
    {
        private static ILog log = LogManager.GetLogger(typeof(LahgiSession));

        
        
        List<TimeSpan> Times;
        List<double> DoseRates;
        List<double> Speeds;
        List<Matrix3D> Poses;
        List<double> Counts;
    }
}
    
