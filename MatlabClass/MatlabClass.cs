using HUREL.ComptonNative;
using System;
using System.Collections.Generic;

namespace HUREL.Compton
{
    public static class MatlabClass 
    {
        public static bool CalcMlemImage(string fileDir, string fileName, bool isPointSet, double maxX, double maxY, double maxZ, double EgateMin, double EgateMax)
        {
           


            var matlabObj = new MatlabFuncs();
            bool testc = (matlabObj.MLEM_Func(fileDir, fileName, isPointSet, maxX, maxY, maxZ, EgateMin, EgateMax) as bool[,])[0, 0];
            if (testc)
            {
                Console.WriteLine("done");
            }

            return true;
        }
    }
}
