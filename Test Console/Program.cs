// See https://aka.ms/new-console-template for more information
using log4net;
using HUREL.Compton;
using System.Diagnostics;

[assembly: log4net.Config.XmlConfigurator(Watch = true)]
namespace LogApp
{

    class Program
    {        
        
        
        static async Task Main(string[] args)
        {
            string status = HUREL.Compton.LahgiApi.StatusMsg;

            Console.WriteLine(status);
            CancellationTokenSource token = new CancellationTokenSource();
            Task? task = null;
            LahgiApi.Echks.Add(new LahgiApi.AddListModeDataEchk(0, 3000));
            if (LahgiApi.IsInitiate)
            {
                task = LahgiApi.StartSessionAsync("test", token);
            }
            Stopwatch sw = Stopwatch.StartNew();
            while(true)
            {
                if (sw.Elapsed > TimeSpan.FromSeconds(20))
                {
                    token.Cancel();
                    break;
                }
            }

            task?.Wait();   
            
            sw.Stop();              
        }




    }
}