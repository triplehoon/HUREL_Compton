using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Diagnostics;

namespace HUREL.Compton
{
    public static class LaccSerialControl
    {
        static SerialPort Serial = new SerialPort();

        public static List<string> PortsName = new List<string>();

        static LaccSerialControl()
        {
            PortsName = new List<string>(SerialPort.GetPortNames());
            if (PortsName == null)
            {
                return;
            }
            SelectedPortName = PortsName[0];
        }
        
        public static void UpdatePortsName()
        {
            PortsName = new List<string>(SerialPort.GetPortNames());
        }

        public static int Baudrate = 9600;
        public static string SelectedPortName;
        public static bool StartCommunication()
        {
            if (Serial.PortName != SelectedPortName)
            {
                Serial.Close();
            }
            if (Serial.IsOpen == true)
            {
                return true;
            }

            Serial.PortName = SelectedPortName;
            Serial.BaudRate = Baudrate;
            Serial.Parity = Parity.None;
            Serial.ReadTimeout = 2000;
            try
            {
                Serial.Open();
            }
            catch (Exception e)
            {
                Trace.WriteLine(e.Message);
                
                return false;
            }
            

            return true;
        }
        public static void StopCommunication()
        {
            if(Serial.IsOpen == false)
            {
                return;
            }
            Serial.Close();
            

        }


        public static void CheckParams()
        {
            if (Serial.IsOpen)
            {
                Serial.WriteLine("check");
                if (Serial.IsOpen)
                {
                    try
                    {
                        ReadCheck(Serial.ReadLine());
                    }
                    catch(Exception e)
                    {
                        Console.WriteLine("Readline Failed:" + e.ToString());
                    }
                }                
            }
        }
        public static void SetFPGA(bool on)
        {
            if (Serial.IsOpen)
            {
                if (on)
                {
                    Serial.WriteLine("setfpga:on");
                }
                else
                {
                    Serial.WriteLine("setfpga:off");
                }
                ReadCheck(Serial.ReadLine());
            }
        }

        public static void SetHvMoudle(bool on)
        {
            if (Serial.IsOpen)
            {
                if (on)
                {
                    Serial.WriteLine("sethv:on");
                }
                else
                {
                    Serial.WriteLine("sethv:off");
                }
                string s = Serial.ReadLine();
                while(s != "done\r")
                {
                    HvModuleVoltage = Convert.ToDouble(s);
                    s = Serial.ReadLine();
                }

                ReadCheck(Serial.ReadLine());

            }
        }

        public static void SetSwitch(int num, bool on)
        {
            if (Serial.IsOpen)
            {
                string s;
                if (on)
                {
                    s = "on";
                }
                else
                {
                    s = "off";
                }

                Serial.WriteLine($"setswitch:{num}:{s}");
                ReadCheck(Serial.ReadLine());
            }

        }
        private static void ReadCheck(string s)
        {
            string[] parameters = s.Split(',');

            if (parameters[0].Split(':')[0] != "hvvolt")
            {
                return;
            }

            HvModuleVoltage = Convert.ToDouble(parameters[0].Split(':')[1]);
            HvModuleCurrent = Convert.ToDouble(parameters[1].Split(':')[1]);
            BatteryVoltage = Convert.ToDouble(parameters[2].Split(':')[1]);
            if (parameters[3].Split(':')[1] == "on")
            {
                IsFPGAOn = true;
            }
            else
            {
                IsFPGAOn = false;
            }

            for (int i = 0; i < 6; ++i)
            {
                if (parameters[4 + i].Split(':')[1] == "on")
                {
                    IsSwitchOn[i] = true;
                }
                else
                {
                    IsSwitchOn[i] = false;
                }
            }


        }

        static public bool IsFPGAOn;
        static public bool[] IsSwitchOn = new bool[6];
        static public double HvModuleVoltage;
        static public double HvModuleCurrent;
        static public double BatteryVoltage;
        


    }
}
