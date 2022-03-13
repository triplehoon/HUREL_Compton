using System;
using HUREL.Compton;

namespace Test_Console
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");
            var envs = Environment.GetEnvironmentVariable("Path");
            string filePath = @"E:\OneDrive - 한양대학교\01.Hurel\01.현재작업\20220311 KOLAS 매틀랩 C# 코드\202203111650Co-60_300sec_0_0_50cm";
            MatlabClass.CalcMlemImage(filePath, "Co-60_300sec_0_0_50cm_ListModeData_TestData.csv",
                false, 0, 0, 0, 1100, 1400);
            
        }
    }
}
