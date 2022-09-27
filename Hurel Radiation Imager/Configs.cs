using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace HUREL.Compton
{
    public static class ConfigValueForCsharp
    {
        static ConfigValueForCsharp()
        {
            
        }

        public static void WriteConfigFile()
        {
            string jsonFilePath = @"Config.json";
            if (!File.Exists(jsonFilePath))
            {
            }
        }
        public static void ReadConfigFile()
        {
            string jsonFilePath = @"Config.json";
  
            if (!File.Exists(jsonFilePath))
            {
                WriteConfigFile();
            }
            //// Json 파일 읽기
            using (StreamReader file = File.OpenText(jsonFilePath))
            using (JsonTextReader reader = new JsonTextReader(file))
            {
                JObject json = (JObject)JToken.ReadFrom(reader);
            }
        }
    }
}
