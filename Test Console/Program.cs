// See https://aka.ms/new-console-template for more information
using log4net;
using HUREL.Compton;

[assembly: log4net.Config.XmlConfigurator(Watch = true)]
namespace LogApp
{

    class Program
    {        
        
        
        static void Main(string[] args)
        {
            string status = HUREL.Compton.LahgiApi.StatusMsg;
            
        }




    }
}