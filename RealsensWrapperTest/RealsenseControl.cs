using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using HUREL.Compton.LACC;

namespace HUREL.Compton.LACC
{
    public class RealsenseControl
    {
        public bool IsInitiate;
        private RealsenseControlWrapper realsenseControl;
        public RealsenseControl(ref string msg)
        {
            realsenseControl = new RealsenseControlWrapper();
            IsInitiate = realsenseControl.InitiateRealsense(ref msg);
        }




    }
}
