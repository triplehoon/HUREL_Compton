using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;



namespace WrapperBase
{
    public class BaseClass
    {
        public BaseClass()
        {
            Mat = new OpenCvSharp.Mat();
        }
        public OpenCvSharp.Mat Mat;
    }
}
