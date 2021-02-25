using CyUSB;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO;
using System.Threading.Tasks.Dataflow;
using System.Threading;
using System.ComponentModel;
using System.Collections.Concurrent;

namespace HUREL.Compton
{
    public partial class CRUXELLLACC  //only cruxell function to make class for Compton GUI
    {

        public BlockingCollection<byte[]> ParsedQueue = new BlockingCollection<byte[]>();
        public BlockingCollection<byte[]> DataInQueue = new BlockingCollection<byte[]>();


        private byte[] DATA_BUFFER; // FEFE제외하고 데이터만 담은 294 data buffer
        private short[] SDATA_BUFFER; // short 버퍼
        //private int DATA_BUFFER_read_count; // 514 DATA_BUFFER 채울때 count
        string INI_PATH = Application.StartupPath + @"\\setting.ini";
        // These are needed to close the app from the Thread exception(exception handling)
        delegate void ExceptionCallback();
        ExceptionCallback handleException;



        private int[] temp_send_buffer = new int[1024];

        int tb_0x11, tb_0x12, tb_0x13, tb_0x14, tb_0x15, tb_0x16, tb_0x17, tb_0x18, tb_0x1a, tb_0x1b, tb_0x0a, tb_0x0b, tb_0x1c;
        int tb_0x19_0, tb_0x19_1, tb_0x19_2, tb_0x19_3, tb_0x19_4, tb_0x19_5, tb_0x19_6, tb_0x19_7, tb_0x19_8, tb_0x19_9, tb_0x19_10, tb_0x19_11, tb_0x19_12, tb_0x19_13, tb_0x19_14, tb_0x19_15;

       


        private Task ListenUBSTask;
        private Task ParsingUSBAsync;
        private Task GenerateShortBufferAsync;

        public bool IsStart;

        private int FlagFinalCall;
        private bool IsListening;

        private bool IsParsing;


        public enum MeasurementMode
        {
            Coincidence = 0,
            Single = 1,
            SingleCoin1 = 2,
            SingleCoin2 = 3
        }

        #region CyUsb Variables
        private int BufSz;
        private int QueueSz;
        private int PPX;
        private int IsoPktBlockSize;
        private int Successes;
        private int Failures;
        //private int p_head = 0; // test2_buffer의 50개중 몇번째인지
        //private int p_tail = 0;
        //private int i_head = 0; // 1메가 배열 내부의 인덱스
        private int p_error = 0;
        private static byte DefaultBufInitValue = 0xA5;
        private DateTime t1, t2;
        private TimeSpan elapsed;
        private double XferBytes;
        private long xferRate;
        private double test_data = 0;

        private CyUSB.USBDeviceList UsbDevices;
        private CyUSBEndPoint EndPoint;
        private int endPointListSelectIdx;
        private int EndPointListSelectIdx
        {
            get 
            {
                return endPointListSelectIdx;
            }
            set
            {
                endPointListSelectIdx = value;
                string sAlt = EndPointList[value].Substring(4, 1);
                byte a = Convert.ToByte(sAlt);
                SelectedDevice.CyDevice.AltIntfc = a;

                // Get the endpoint
                int aX = EndPointList[value].LastIndexOf("0x");
                string sAddr = EndPointList[value].Substring(aX, 4);
                byte addr = (byte)Util.HexToInt(sAddr);

                EndPoint = SelectedDevice.CyDevice.EndPointOf(addr);

                // Ensure valid PPX for this endpoint
                Ppx_SelectedIndexChanged();
            }
        }

        //------------------------------+---------------------------------------------------------------
        // [함수] USB 관련 함수 | This is the System event handler.
        //                        Enforces valid values for PPX(Packet per transfer)
        //------------------------------+---------------------------------------------------------------
        private void Ppx_SelectedIndexChanged()
        {
            if (EndPoint == null) return;

            int ppx = PPX;
            int len = EndPoint.MaxPktSize * ppx;

            int maxLen = 0x400000; // 4MBytes
            if (len > maxLen)
            {
                //ppx = maxLen / (EndPoint.MaxPktSize) / 8 * 8;
                if (EndPoint.MaxPktSize == 0)
                {
                    MessageBox.Show("Please correct MaxPacketSize in Descriptor", "Invalid MaxPacketSize");
                    return;
                }
                ppx = maxLen / (EndPoint.MaxPktSize);
                ppx -= (ppx % 8);
                MessageBox.Show("Maximum of 4MB per transfer.  Packets reduced.", "Invalid Packets per Xfer.");

                //Update the DropDown list for the packets
                int iIndex = PPX; // Get the packet index
                //PpxBox.Items.Remove(PpxBox.Text); // Remove the Existing  Packet index
                //PpxBox.Items.Insert(iIndex, ppx.ToString()); // insert the ppx
                //PpxBox.SelectedIndex = iIndex; // update the selected item index

            }


            if ((SelectedDevice.CyDevice.bSuperSpeed || SelectedDevice.CyDevice.bHighSpeed) && (EndPoint.Attributes == 1) && (ppx < 8))
            {
                PPX = 8;
                MessageBox.Show("Minimum of 8 Packets per Xfer required for HS/SS Isoc.", "Invalid Packets per Xfer.");
            }
            if ((SelectedDevice.CyDevice.bHighSpeed) && (EndPoint.Attributes == 1))
            {
                if (ppx > 128)
                {
                    PPX = 128;
                    MessageBox.Show("Maximum 128 packets per transfer for High Speed Isoc", "Invalid Packets per Xfer.");
                }
            }

        }

        private List<string> EndPointList = new List<string>();
        private int PpxInfo = 0;
        private int QueueInfo = 0;
        public DeviceInfo SelectedDevice;
        public List<DeviceInfo> DeviceList = new List<DeviceInfo>();
        public class DeviceInfo
        {
            public USBDevice Device { get; set; }
            public CyUSBDevice CyDevice { get; set; }
            public string DeviceName { get; set; }
        }
        #endregion

        #region Setting Variables
        public bool IsVariablesSet = false;
        public VariableInfo Variables = new VariableInfo();
        public class VariableInfo
        {
            public int RecordTime0x0a;
            public int RecordCount0x0b = 1;
            public MeasurementMode CurrentMeasurementMode0x11 =MeasurementMode.Coincidence;
            public int GlobalTriggerWindow0x12 = -5000;
            public int SmoothWindow0x13;
            public int BaseLineInterval0x14;
            public int BaseLineOffset0x15;
            public int MaxOffset0x16;
            public int MaxMeasInterval0x17;
            public int TriggerSlopePoints0x18;
            public int[] TriggerThreshold0x19 = new int[16];
            public int NoTrgforDetTrg0x1a;
            public int TWforDetTrg0x1b;
            /// <summary>
            /// Always set as 0
            /// </summary>
            public int Transfersize0x1c
            {
                get {
                    return 0;
                }
            }
            public int AverageRun0x1d;
            public string FileName;
        }
        public bool SetVaribles(VariableInfo variables)
        {
           if(variables.GlobalTriggerWindow0x12 == -5000)
            {
                return false; 
            }
           Variables = variables;            
           IsVariablesSet = true;
           return true;
        }
        #endregion

        #region DAQ varibales
        public string FileMainPath;
        #endregion

        public event EventHandler USBChangeHandler;
        public void USBChange()
        {
            if (USBChangeHandler != null)
            {
                USBChangeHandler(this, EventArgs.Empty);
            }
        }

        /// <summary>
        /// Contructor. Initiate init_data_and_buffer and an ini file.
        /// </summary>
        public CRUXELLLACC()
        {
            Init_data_and_buffer();
            Init_USB();
        }

        public async Task Dispose()
        {
            IsListening = false;
            IsParsing = false;
            if (DeviceList.Count != 0)
            { }
                //UsbDevices.Dispose();

            Write_current_ini(); // 현재 설정 저장하기
            if(IsStart)
            {
                ListenUBSTask.GetAwaiter().GetResult();
                await ParsingUSBAsync;
                await GenerateShortBufferAsync;
            }

            Trace.WriteLine("HY : EXIT");
        }


        public bool Start_usb(out string status)
        {
            if (!IsVariablesSet)
            {
                status = "Variable is not set";
                return false;
            }
            

            // 사장님 - Start 눌렀을때
            if (SelectedDevice.DeviceName == "")
            {
                Trace.WriteLine("No selected Devices");
                status = "No selected Devices";
                return false;
            }


            if (QueueInfo == 0)
            {
                Trace.WriteLine("Please Select Xfers to Queue Invalid Input");
                status = "Please Select Xfers to Queue Invalid Input";
                return false;
            }

            if (!IsVariablesSet)
            {
                Trace.WriteLine("Please Set Variables");
                status = "Please Set Variables";
                return false;
            }

            // Test GetData from box
            get_data_from_varialbes();




            Trace.WriteLine("HY : Start");
            // File save 작업
            Trace.WriteLine("HY : [Try] init_file_save ");
            init_file_save_bin();

            // 1. 입력칸 비활성화 및 설정
            Trace.WriteLine("HY : [Try] input_disable ");

            byte[] Item;

            while (DataInQueue.TryTake(out Item))
            {
            }

            Trace.WriteLine("HY : [Try] Send 0000... ");
            usb_setting(3); // send 00000

            // 2. 버퍼 비우기 
            Trace.WriteLine("HY : [Try] XferData reset loop");
            EndPointListSelectIdx = 1;
            EndPoint.TimeOut = 500;

            bool bResult = true;
            int xferLen = 4096;
            byte[] inData = new byte[xferLen];
            while (bResult)
            {
                bResult = EndPoint.XferData(ref inData, ref xferLen);
            }
            EndPoint.TimeOut = 500;

            Trace.WriteLine("HY : [Try] send setting value");
            // 3. 셋팅값 전송
            usb_setting(1);

            // 4. 데이터 Read
            EndPointListSelectIdx = 1;

            BufSz = EndPoint.MaxPktSize * PpxInfo;
            p_error = EndPoint.MaxPktSize; // 16384
            QueueSz = QueueInfo; // 값 1로 고정
            PPX = PpxInfo;

            EndPoint.XferSize = BufSz;

            if (EndPoint is CyIsocEndPoint)
                IsoPktBlockSize = (EndPoint as CyIsocEndPoint).GetPktBlockSize(BufSz);
            else
                IsoPktBlockSize = 0;






            IsListening = true;
            FlagFinalCall = 0;
            Debug.WriteLine("HY : [Try] Start XferThread");
            //ListenUSBThread = new Thread(new ThreadStart( XferThread));
            //ListenUSBThread.Start();
            ListenUBSTask = Task.Run(() => XferThread());

            IsParsing = true;
            Debug.WriteLine("HY : [Try] Start ParsingThread");
            ParsingUSBAsync = Task.Run(() => ParsingCyusbBuffer());

            IsGenerateShortArrayBuffer = true;
            Debug.WriteLine("HY : [Try] Start Generate Short Array Buffer");
            GenerateShortBufferAsync = Task.Run(() => GenerateShortArrayBuffer());
            status = "Data Aquisition Start";
            IsStart = true;

            
            Debug.WriteLine("HY: FPGA Start Setup Done");
            return true;
        }
        public async Task<string> Stop_usb()
        {
            if (!IsStart)
            {
                return "Have to start usb";
            }
            IsStart = false;
            Debug.WriteLine("wait for ListenUBSAsync");
            IsListening = false;
            ListenUBSTask.GetAwaiter().GetResult();
            ListenUBSTask = null;
            
            IsParsing = false;
            Debug.WriteLine("wait for tParsing");
            await ParsingUSBAsync;
            ParsingUSBAsync = null;

            IsGenerateShortArrayBuffer = false;
            await GenerateShortBufferAsync;

            usb_setting(3); // 모든처리 끝났을때 stop

           // DATA_BUFFER_read_count = 0;

            return "Done";
        }


        #region initiate Setting
        private void Init_data_and_buffer()
        {
            // 1. 버퍼 초기화
            DATA_BUFFER = new byte[294];
            SDATA_BUFFER = new short[147];

            // 2. 초기값 설정 (ini 확인 후 있으면 적용, 없으면 작성)
            if (System.IO.File.Exists(INI_PATH))
            {
                Read_current_ini();
            }
            else
            {
                Set_first_setting();
                Write_current_ini();
            }
        }
        private void Write_current_ini()
        {
            WriteINI("setting", "GlobalTriggerWindow0x12", Variables.GlobalTriggerWindow0x12.ToString());
            WriteINI("setting", "SmoothWindow0x13", Variables.SmoothWindow0x13.ToString());
            WriteINI("setting", "BaseLineInterval0x14", Variables.BaseLineInterval0x14.ToString());
            WriteINI("setting", "BaseLineOffset0x15", Variables.BaseLineOffset0x15.ToString());
            WriteINI("setting", "MaxOffset0x16", Variables.MaxOffset0x16.ToString());
            WriteINI("setting", "MaxMeasInterval0x17", Variables.MaxMeasInterval0x17.ToString());
            WriteINI("setting", "TriggerSlopePoints0x18", Variables.TriggerSlopePoints0x18.ToString());
            WriteINI("setting", "TriggerThreshold0x19[0]", Variables.TriggerThreshold0x19[0].ToString());
            WriteINI("setting", "TriggerThreshold0x19[1]", Variables.TriggerThreshold0x19[1].ToString());
            WriteINI("setting", "TriggerThreshold0x19[2]", Variables.TriggerThreshold0x19[2].ToString());
            WriteINI("setting", "TriggerThreshold0x19[3]", Variables.TriggerThreshold0x19[3].ToString());
            WriteINI("setting", "TriggerThreshold0x19[4]", Variables.TriggerThreshold0x19[4].ToString());
            WriteINI("setting", "TriggerThreshold0x19[5]", Variables.TriggerThreshold0x19[5].ToString());
            WriteINI("setting", "TriggerThreshold0x19[6]", Variables.TriggerThreshold0x19[6].ToString());
            WriteINI("setting", "TriggerThreshold0x19[7]", Variables.TriggerThreshold0x19[7].ToString());
            WriteINI("setting", "TriggerThreshold0x19[8]", Variables.TriggerThreshold0x19[8].ToString());
            WriteINI("setting", "TriggerThreshold0x19[9]", Variables.TriggerThreshold0x19[9].ToString());
            WriteINI("setting", "TriggerThreshold0x19[10]", Variables.TriggerThreshold0x19[10].ToString());
            WriteINI("setting", "TriggerThreshold0x19[11]", Variables.TriggerThreshold0x19[11].ToString());
            WriteINI("setting", "TriggerThreshold0x19[12]", Variables.TriggerThreshold0x19[12].ToString());
            WriteINI("setting", "TriggerThreshold0x19[13]", Variables.TriggerThreshold0x19[13].ToString());
            WriteINI("setting", "TriggerThreshold0x19[14]", Variables.TriggerThreshold0x19[14].ToString());
            WriteINI("setting", "TriggerThreshold0x19[15]", Variables.TriggerThreshold0x19[15].ToString());

            WriteINI("setting", "NoTrgforDetTrg0x1a", Variables.NoTrgforDetTrg0x1a.ToString());
            WriteINI("setting", "TWforDetTrg0x1b", Variables.TWforDetTrg0x1b.ToString());
            WriteINI("setting", "Transfersize0x1c", Variables.Transfersize0x1c.ToString());
            WriteINI("setting", "AverageRun0x1d", Variables.AverageRun0x1d.ToString());
            switch (Variables.CurrentMeasurementMode0x11)
            {
                case MeasurementMode.Coincidence:
                    WriteINI("setting", "CurrentMeasurementMode0x11", "Single");
                    break;

                case MeasurementMode.Single:
                    WriteINI("setting", "CurrentMeasurementMode0x11", "Coin");
                    break;

                case MeasurementMode.SingleCoin1:
                    WriteINI("setting", "CurrentMeasurementMode0x11", "CS1");
                    break;

                case MeasurementMode.SingleCoin2:
                    WriteINI("setting", "CurrentMeasurementMode0x11", "CS2");
                    break;
            }
        }
        private void Read_current_ini()
        {
            Variables.GlobalTriggerWindow0x12 = Convert.ToInt32(ReadINI("setting", "GlobalTriggerWindow0x12"));
            Variables.SmoothWindow0x13 = Convert.ToInt32(ReadINI("setting", "SmoothWindow0x13"));
            Variables.BaseLineInterval0x14 = Convert.ToInt32(ReadINI("setting", "BaseLineInterval0x14"));
            Variables.BaseLineOffset0x15 = Convert.ToInt32(ReadINI("setting", "BaseLineOffset0x15"));
            Variables.MaxOffset0x16 = Convert.ToInt32(ReadINI("setting", "MaxOffset0x16"));
            Variables.MaxMeasInterval0x17 = Convert.ToInt32(ReadINI("setting", "MaxMeasInterval0x17"));
            Variables.TriggerSlopePoints0x18 = Convert.ToInt32(ReadINI("setting", "TriggerSlopePoints0x18"));
            Variables.TriggerThreshold0x19 = new int[16];
            for (int i = 0; i < Variables.TriggerThreshold0x19.Length; i++)
            {
                string variableName = "TriggerThreshold0x19[" + i + "]";
                Variables.TriggerThreshold0x19[i] = Convert.ToInt32(ReadINI("setting", variableName));
            }
            Variables.NoTrgforDetTrg0x1a = Convert.ToInt32(ReadINI("setting", "NoTrgforDetTrg0x1a"));
            Variables.TWforDetTrg0x1b = Convert.ToInt32(ReadINI("setting", "TWforDetTrg0x1b"));
            //Variables.Transfersize0x1c = Convert.ToInt32(ReadINI("setting", "Transfersize0x1c"));
            Variables.AverageRun0x1d = Convert.ToInt32(ReadINI("setting", "AverageRun0x1d"));

            string mode = ReadINI("setting", "CurrentMeasurementMode0x11");
            switch (mode)
            {
                case "Single":
                    Variables.CurrentMeasurementMode0x11 = MeasurementMode.Coincidence;
                    WriteINI("setting", "CurrentMeasurementMode0x11", "Single");
                    break;

                case "Coin":
                    Variables.CurrentMeasurementMode0x11 = MeasurementMode.Single;
                    WriteINI("setting", "CurrentMeasurementMode0x11", "Coin");
                    break;

                case "CS1":
                    Variables.CurrentMeasurementMode0x11 = MeasurementMode.SingleCoin1;
                    WriteINI("setting", "CurrentMeasurementMode0x11", "CS1");
                    break;

                case "CS2":
                    Variables.CurrentMeasurementMode0x11 = MeasurementMode.SingleCoin2;
                    WriteINI("setting", "CurrentMeasurementMode0x11", "CS2");
                    break;
            }

            IsVariablesSet = true;
        }        

        [DllImport("kernel32")]
        private static extern long WritePrivateProfileString(string section, string key, string val, string filePath);
        [DllImport("kernel32")]
        private static extern long GetPrivateProfileString(string section, string key, string def, StringBuilder retVal, int sise, string filePath);
        private string ReadINI(string section, string key)
        {
            StringBuilder sb = new StringBuilder(255);
            GetPrivateProfileString(section, key, "", sb, sb.Capacity, INI_PATH);
            return sb.ToString();
        }
        private void WriteINI(string section, string key, string value)
        {
            WritePrivateProfileString(section, key, value, INI_PATH);
        }
        private void Set_first_setting()
        {
            Variables.CurrentMeasurementMode0x11 = MeasurementMode.Single;
            Variables.GlobalTriggerWindow0x12 = 0;
            Variables.SmoothWindow0x13 = 0;
            Variables.BaseLineInterval0x14 = 0;
            Variables.BaseLineOffset0x15 = 0;
            Variables.MaxOffset0x16 = 0;
            Variables.MaxMeasInterval0x17 = 0;
            Variables.TriggerSlopePoints0x18 = 0;
            Variables.TriggerThreshold0x19 = new int[16];
            Variables.NoTrgforDetTrg0x1a = 0;
            Variables.TWforDetTrg0x1b = 0;
            //Variables.Transfersize0x1c = 0;
        }
        private void Init_USB()
        {
            // Setup the callback routine for NullReference exception handling
            handleException = new ExceptionCallback(ThreadException);

            // Create the list of USB devices attached to the CyUSB3.sys driver.
            try
            {
                UsbDevices = new USBDeviceList(CyConst.DEVICES_CYUSB);
            }
            catch { }
            SelectedDevice = new DeviceInfo(); 
            
            //Assign event handlers for device attachment and device removal.
            UsbDevices.DeviceAttached += new EventHandler(usbDevices_DeviceAttached);
            UsbDevices.DeviceRemoved += new EventHandler(usbDevices_DeviceRemoved);

            //Set and search the device with VID-PID 04b4-1003 and if found, selects the end point
            SetDevice(false);
        }
       
        /// <summary>
        /// [함수] USB 관련 함수 | This is the event handler for device attachment. This method  searches for the device with VID-PID 04b4-00F1
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void usbDevices_DeviceAttached(object sender, EventArgs e)
        {
            SetDevice(false);
        }

        /// <summary>
        /// [함수] USB 관련 함수 | This is the event handler for device removal. This method resets the device count and searches for the device with VID-PID 04b4-1003
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void usbDevices_DeviceRemoved(object sender, EventArgs e)
        {
            IsListening = false;
            if (ListenUBSTask != null)
            {
                ListenUBSTask.GetAwaiter().GetResult();

            }
            SelectedDevice = new DeviceInfo(); ;
            EndPoint = null;
            DeviceList.Clear();
            SetDevice(false);
            
        }

        /// <summary>
        /// [함수] USB 관련 함수 | Search the device with VID-PID 04b4-00F1 and if found, select the end point
        /// </summary>
        /// <param name="bPreserveSelectedDevice"></param>
        private void SetDevice(bool bPreserveSelectedDevice)
        {           
            int nDeviceList = UsbDevices.Count;
            for (int nCount = 0; nCount < nDeviceList; nCount++)
            {
                USBDevice fxDevice = UsbDevices[nCount];
                String strmsg;
                strmsg = "(0x" + fxDevice.VendorID.ToString("X4") + " - 0x" + fxDevice.ProductID.ToString("X4") + ") " + fxDevice.FriendlyName;
                DeviceInfo deviceInfo = new DeviceInfo { Device = fxDevice, DeviceName = strmsg };
                DeviceList.Add(deviceInfo);
            }

            if (DeviceList.Count > 0)
                SelectedDevice = ((bPreserveSelectedDevice == true) ? SelectedDevice : DeviceList[0]);

            USBDevice dev = SelectedDevice.Device;

            if (dev != null)
            {
                SelectedDevice.CyDevice = (CyUSBDevice)dev;

                GetEndpointsOfNode(SelectedDevice.CyDevice.Tree);
                PpxInfo = 64; //Set default value to 8 Packets
                QueueInfo = 128; //128
                if (EndPointList.Count > 0)
                {
                    EndPointListSelectIdx = 0;                    
                }          
            }
            else
            {

                EndPointList.Clear();
            }
            USBChange();
        }

        /// <summary>
        /// [함수] USB 관련 함수 | Recursive routine populates EndPointList with strings representing all the endpoints in the device.
        ///  </summary>                      
        /// <param name="devTree"></param>
        private void GetEndpointsOfNode(TreeNode devTree)
        {
            EndPointList.Clear(); // 이거 주석되있어서 계속추가되는 버그있던데, 예제소스가 왜그런진 모르겠음
            foreach (TreeNode node in devTree.Nodes)
            {
                if (node.Nodes.Count > 0)
                    GetEndpointsOfNode(node);
                else
                {
                    CyUSBEndPoint ept = node.Tag as CyUSBEndPoint;
                    if (ept == null)
                    {
                        //return;
                    }
                    else if (!node.Text.Contains("Control"))
                    {
                        CyUSBInterface ifc = node.Parent.Tag as CyUSBInterface;
                        string s = string.Format("ALT-{0}, {1} Byte {2}", ifc.bAlternateSetting, ept.MaxPktSize, node.Text);
                        EndPointList.Add(s);
                    } 
                }
            }
        }

        /// <summary>
        /// [함수] USB 관련 함수 | The callback routine delegated to handleException.
        /// </summary>
        public void ThreadException()
        {
            IsListening = false;
            ListenUBSTask = null;
        }
        #endregion

        #region Data Aqusition

        private void init_file_save_bin()
        {
            string currentTime = DateTime.Now.ToString("yyyyMMddHHmm");
            // .bin 저장을 위한 경로
            string file_path = Application.StartupPath + "\\" + currentTime + Variables.FileName;
            DirectoryInfo di = new DirectoryInfo(file_path);
            if (di.Exists == false) // if new folder not exits
            {
                di.Create(); // create Folder
                FileMainPath = (file_path + "\\" + currentTime + Variables.FileName + ".bin");
            }
            else // if exits
            {
                int file_name_counter = 0;

                while (true)
                {
                    string new_file_path = Application.StartupPath + "\\" + currentTime + Variables.FileName + $"_{file_name_counter}";
                    di = new DirectoryInfo(new_file_path);
                    if (di.Exists == false)
                    {
                        di.Create(); // create Folder
                        FileMainPath = (new_file_path + "\\" + currentTime + Variables.FileName + $"_{file_name_counter}" + ".bin");
                        break;
                    }
                    else
                    {
                        file_name_counter++;
                    }
                }
            }
        }

        private void usb_setting(int flag)
        {
            // 1. out endpoint로 설정
            EndPointListSelectIdx = 0;

            // 2. USB endpoint 연결
            int outData_BufSz = 4096;

            EndPoint.XferSize = outData_BufSz;

            if (EndPoint is CyIsocEndPoint)
                IsoPktBlockSize = (EndPoint as CyIsocEndPoint).GetPktBlockSize(outData_BufSz);
            else
                IsoPktBlockSize = 0;

            byte[] outData = new byte[outData_BufSz];
            if (flag == 1)
            {
                // 최초 셋팅값 전송
                tryparse_send(ref outData, ref outData_BufSz, tb_0x0a, 0x0a); // Sec
                tryparse_send(ref outData, ref outData_BufSz, tb_0x0b, 0x0b); // Count
                tryparse_send(ref outData, ref outData_BufSz, tb_0x11, 0x11); // Single / Coincidence Mode
                tryparse_send(ref outData, ref outData_BufSz, tb_0x12, 0x12); // T.W for Global Trig
                tryparse_send(ref outData, ref outData_BufSz, tb_0x13, 0x13); // Smooth Window
                tryparse_send(ref outData, ref outData_BufSz, tb_0x14, 0x14); // BL.Meas.Interval
                tryparse_send(ref outData, ref outData_BufSz, tb_0x15, 0x15); // BL.Offset
                tryparse_send(ref outData, ref outData_BufSz, tb_0x16, 0x16); // Max Offset
                tryparse_send(ref outData, ref outData_BufSz, tb_0x17, 0x17); // Max Meas.Interval
                tryparse_send(ref outData, ref outData_BufSz, tb_0x18, 0x18); // Trig.Slope Points
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_0, 0x19, 0x01); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_1, 0x19, 0x02); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_2, 0x19, 0x03); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_3, 0x19, 0x04); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_4, 0x19, 0x05); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_5, 0x19, 0x06); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_6, 0x19, 0x07); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_7, 0x19, 0x08); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_8, 0x19, 0x09); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_9, 0x19, 0x0a); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_10, 0x19, 0x0b); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_11, 0x19, 0x0c); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_12, 0x19, 0x0d); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_13, 0x19, 0x0e); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_14, 0x19, 0x0f); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x19_15, 0x19, 0x10); // Local Trig.Threshold   
                tryparse_send(ref outData, ref outData_BufSz, tb_0x1a, 0x1a); // No.Trig.for Det Trig
                tryparse_send(ref outData, ref outData_BufSz, tb_0x1b, 0x1b); // T.W for Det.Trig


                tryparse_send(ref outData, ref outData_BufSz, tb_0x1c, 0x1c); // For DEBUG
                tryparse_send(ref outData, ref outData_BufSz, 1, 0x1d); // FIFO CLR Always 1


                // 런신호 (768 / 770)
                reset_send_buffer(ref outData, ref outData_BufSz);
                temp_send_buffer[768] = 1;
                temp_send_buffer[770] = 1;

                Buffer.BlockCopy(temp_send_buffer, 0, outData, 0, temp_send_buffer.Length * 4);

                EndPoint.TimeOut = 10;
                for (int i = 0; i < 1000; ++i)
                {
                    if (EndPoint.XferData(ref outData, ref outData_BufSz))
                        break;
                    Thread.Sleep(10);
                }
            }
            else if (flag == 2)
            {
                // Final call (768 / 770)
                reset_send_buffer(ref outData, ref outData_BufSz);
                temp_send_buffer[768] = 1;
                temp_send_buffer[769] = 1;
                temp_send_buffer[770] = 1;

                Buffer.BlockCopy(temp_send_buffer, 0, outData, 0, temp_send_buffer.Length * 4);

                EndPoint.TimeOut = 10;
                int i = 0;
                for (i = 0; i < 100; ++i)
                {
                    if (EndPoint.XferData(ref outData, ref outData_BufSz))
                        break;
                    Thread.Sleep(10);
                }
                Trace.WriteLine($"HY : flag2 {i}");
            }
            else if (flag == 3)
            {
                Trace.WriteLine("HY : [Try] Reset_send_buffer [3]");
                // Stop (전부 0)
                reset_send_buffer(ref outData, ref outData_BufSz);

                EndPoint.TimeOut = 1;
                int i = 0;
                for (i = 0; i < 1000; ++i)
                {
                    if (EndPoint.XferData(ref outData, ref outData_BufSz))
                        break;
                    Thread.Sleep(0);
                }
                Trace.WriteLine($"HY : [Try] 0000 send done : {i}");
            }
            EndPointListSelectIdx = 1;
        }


        private void get_data_from_varialbes()
        {
            switch (Variables.CurrentMeasurementMode0x11)
            {
                case MeasurementMode.Coincidence:
                    tb_0x11 = 0;
                    break;
                case MeasurementMode.Single:
                    tb_0x11 = 1;
                    break;
                case MeasurementMode.SingleCoin1:
                    tb_0x11 = 2;
                    break;
                case MeasurementMode.SingleCoin2:
                    tb_0x11 = 3;
                    break;
            }

            tb_0x12 = Variables.GlobalTriggerWindow0x12;
            tb_0x13 = Variables.SmoothWindow0x13;
            tb_0x14 = Variables.BaseLineInterval0x14;
            tb_0x15 = Variables.BaseLineOffset0x15;
            tb_0x16 = Variables.MaxOffset0x16;
            tb_0x17 = Variables.MaxMeasInterval0x17;
            tb_0x18 = Variables.TriggerSlopePoints0x18;

            tb_0x19_0 = Variables.TriggerThreshold0x19[0];
            tb_0x19_1 = Variables.TriggerThreshold0x19[1];
            tb_0x19_2 = Variables.TriggerThreshold0x19[2];
            tb_0x19_3 = Variables.TriggerThreshold0x19[3];
            tb_0x19_4 = Variables.TriggerThreshold0x19[4];
            tb_0x19_5 = Variables.TriggerThreshold0x19[5];
            tb_0x19_6 = Variables.TriggerThreshold0x19[6];
            tb_0x19_7 = Variables.TriggerThreshold0x19[7];
            tb_0x19_8 = Variables.TriggerThreshold0x19[8];
            tb_0x19_9 = Variables.TriggerThreshold0x19[9];
            tb_0x19_10= Variables.TriggerThreshold0x19[10];
            tb_0x19_11= Variables.TriggerThreshold0x19[11];
            tb_0x19_12= Variables.TriggerThreshold0x19[12];
            tb_0x19_13= Variables.TriggerThreshold0x19[13];
            tb_0x19_14= Variables.TriggerThreshold0x19[14];
            tb_0x19_15= Variables.TriggerThreshold0x19[15];
            tb_0x1a = Variables.NoTrgforDetTrg0x1a;
            tb_0x1b = Variables.TWforDetTrg0x1b;
            tb_0x0a = Variables.RecordTime0x0a;
            tb_0x0b = Variables.RecordCount0x0b;
            tb_0x1c = Variables.Transfersize0x1c;


            if (tb_0x0a * tb_0x0b == 0) // 둘중하나라도 0이있으면, 둘다 0이면
            {
                tb_0x0a = 0;
                tb_0x0b = 0;
            }   
        }

        /// <summary>
        ///  [함수] 최초 셋팅 값 한번만 보내기 용
        /// </summary>
        private void reset_send_buffer(ref byte[] outData, ref int outData_BufSz)
        {
            for (int i = 0; i < 1024; ++i)
                temp_send_buffer[i] = 0;
            for (int i = 0; i < outData_BufSz; ++i)
                outData[i] = 0;
        }

        private unsafe void tryparse_send(ref byte[] outData, ref int outData_BufSz, int data, int add, int sub_add = -1)
        {
            reset_send_buffer(ref outData, ref outData_BufSz);

            int get_data = data;

            for (int i = 0; i < 700; ++i)
            {
                if (sub_add != -1)
                    temp_send_buffer[i] = (add << 24) | (sub_add << 16) | (get_data & 0xFFFF);
                else
                    temp_send_buffer[i] = (add << 24) | (get_data & 0xFFFFFF);
            }



            Buffer.BlockCopy(temp_send_buffer, 0, outData, 0, temp_send_buffer.Length * 4);

            EndPoint.TimeOut = 10;
            for (int i = 0; i < 1000; ++i)
            {
                if (EndPoint.XferData(ref outData, ref outData_BufSz))
                    break;
                Thread.Sleep(10);
            }
        }

        /// <summary>
        /// [함수] USB 관련 함수 | Data Xfer Thread entry point. Starts the thread on Start
        /// </summary>
        public unsafe void XferThread()
        {
            // Setup the queue buffers
            byte[][] cmdBufs = new byte[QueueSz][];
            byte[][] xferBufs = new byte[QueueSz][];
            byte[][] ovLaps = new byte[QueueSz][];
            ISO_PKT_INFO[][] pktsInfo = new ISO_PKT_INFO[QueueSz][];

            //int xStart = 0;

            //////////////////////////////////////////////////////////////////////////////
            ///////////////Pin the data buffer memory, so GC won't touch the memory///////
            //////////////////////////////////////////////////////////////////////////////

            GCHandle cmdBufferHandle = GCHandle.Alloc(cmdBufs[0], GCHandleType.Pinned);
            GCHandle xFerBufferHandle = GCHandle.Alloc(xferBufs[0], GCHandleType.Pinned);
            GCHandle overlapDataHandle = GCHandle.Alloc(ovLaps[0], GCHandleType.Pinned);
            GCHandle pktsInfoHandle = GCHandle.Alloc(pktsInfo[0], GCHandleType.Pinned);

            try
            {
                LockNLoad(cmdBufs, xferBufs, ovLaps, pktsInfo);
            }
            catch (NullReferenceException e)
            {
                // This exception gets thrown if the device is unplugged 
                // while we're streaming data
                e.GetBaseException();
                //this.Invoke(handleException);
            }
           
            //////////////////////////////////////////////////////////////////////////////
            ///////////////Release the pinned memory and make it available to GC./////////
            //////////////////////////////////////////////////////////////////////////////
            cmdBufferHandle.Free();
            xFerBufferHandle.Free();
            overlapDataHandle.Free();
            pktsInfoHandle.Free();
        }

        /// <summary>
        /// [함수] USB 관련 함수 |
        /// This is a recursive routine for pinning all the buffers used in the transfer in memory.
        /// It will get recursively called QueueSz times.On the QueueSz_th call, it will call
        /// XferData, which will loop, transferring data, until the stop button is clicked.
        /// Then, the recursion will unwind.
        /// </summary>
        /// <param name="cBufs"></param>
        /// <param name="xBufs"></param>
        /// <param name="oLaps"></param>
        /// <param name="pktsInfo"></param>
        public unsafe void LockNLoad(byte[][] cBufs, byte[][] xBufs, byte[][] oLaps, ISO_PKT_INFO[][] pktsInfo)
        {
            int j = 0;
            int nLocalCount = j;

            GCHandle[] bufSingleTransfer = new GCHandle[QueueSz];
            GCHandle[] bufDataAllocation = new GCHandle[QueueSz];
            GCHandle[] bufPktsInfo = new GCHandle[QueueSz];
            GCHandle[] handleOverlap = new GCHandle[QueueSz];

            while (j < QueueSz)
            {
                // Allocate one set of buffers for the queue, Buffered IO method require user to allocate a buffer as a part of command buffer,
                // the BeginDataXfer does not allocated it. BeginDataXfer will copy the data from the main buffer to the allocated while initializing the commands.
                cBufs[j] = new byte[CyConst.SINGLE_XFER_LEN + IsoPktBlockSize + ((EndPoint.XferMode == XMODE.BUFFERED) ? BufSz : 0)];

                xBufs[j] = new byte[BufSz];

                //initialize the buffer with initial value 0xA5
                for (int iIndex = 0; iIndex < BufSz; iIndex++)
                    xBufs[j][iIndex] = DefaultBufInitValue;

                int sz = Math.Max(CyConst.OverlapSignalAllocSize, sizeof(OVERLAPPED));
                oLaps[j] = new byte[sz];
                pktsInfo[j] = new ISO_PKT_INFO[PPX];

                /*/////////////////////////////////////////////////////////////////////////////
                 * 
                 * fixed keyword is getting thrown own by the compiler because the temporary variables 
                 * tL0, tc0 and tb0 aren't used. And for jagged C# array there is no way, we can use this 
                 * temporary variable.
                 * 
                 * Solution  for Variable Pinning:
                 * Its expected that application pin memory before passing the variable address to the
                 * library and subsequently to the windows driver.
                 * 
                 * Cypress Windows Driver is using this very same memory location for data reception or
                 * data delivery to the device.
                 * And, hence .Net Garbage collector isn't expected to move the memory location. And,
                 * Pinning the memory location is essential. And, not through FIXED keyword, because of 
                 * non-usability of temporary variable.
                 * 
                /////////////////////////////////////////////////////////////////////////////*/
                //fixed (byte* tL0 = oLaps[j], tc0 = cBufs[j], tb0 = xBufs[j])  // Pin the buffers in memory
                //////////////////////////////////////////////////////////////////////////////////////////////
                bufSingleTransfer[j] = GCHandle.Alloc(cBufs[j], GCHandleType.Pinned);
                bufDataAllocation[j] = GCHandle.Alloc(xBufs[j], GCHandleType.Pinned);
                bufPktsInfo[j] = GCHandle.Alloc(pktsInfo[j], GCHandleType.Pinned);
                handleOverlap[j] = GCHandle.Alloc(oLaps[j], GCHandleType.Pinned);
                // oLaps "fixed" keyword variable is in use. So, we are good.
                /////////////////////////////////////////////////////////////////////////////////////////////            

                unsafe
                {
                    //fixed (byte* tL0 = oLaps[j])
                    {
                        CyUSB.OVERLAPPED ovLapStatus = new CyUSB.OVERLAPPED();
                        ovLapStatus = (CyUSB.OVERLAPPED)Marshal.PtrToStructure(handleOverlap[j].AddrOfPinnedObject(), typeof(CyUSB.OVERLAPPED));
                        ovLapStatus.hEvent = (IntPtr)PInvoke.CreateEvent(0, 0, 0, 0);
                        Marshal.StructureToPtr(ovLapStatus, handleOverlap[j].AddrOfPinnedObject(), true);

                        // Pre-load the queue with a request
                        int len = BufSz;
                        if (EndPoint.BeginDataXfer(ref cBufs[j], ref xBufs[j], ref len, ref oLaps[j]) == false)
                        {
                            Failures++;// Failures++;
                        }
                           
                    }
                    j++;
                }
            }
            try
            {
                XferData(cBufs, xBufs, oLaps, pktsInfo, handleOverlap);          // All loaded. Let's go!
            }
            catch (CyUSBBufferFailException e)
            {
                Debug.WriteLine(e.Message);
                Trace.WriteLine("XferData Fail, Restart UBS");
                unsafe
                {
                    //Trace.WriteLine("HY : [Try] TryTake loop(test_buffer reset)");
                    while (true)
                    {
                        int testBuffcount = DataInQueue.Count;
                        if (testBuffcount != 0)
                        {
                            Thread.Sleep(0);
                            //Debug.WriteLine("Remaining test_buffer count is " + testBuffcount);
                        }
                        else
                            break;
                    }

                    for (nLocalCount = 0; nLocalCount < QueueSz; nLocalCount++)
                    {
                        CyUSB.OVERLAPPED ovLapStatus = new CyUSB.OVERLAPPED();
                        ovLapStatus = (CyUSB.OVERLAPPED)Marshal.PtrToStructure(handleOverlap[nLocalCount].AddrOfPinnedObject(), typeof(CyUSB.OVERLAPPED));
                        PInvoke.CloseHandle(ovLapStatus.hEvent);

                        /*////////////////////////////////////////////////////////////////////////////////////////////
                         * 
                         * Release the pinned allocation handles.
                         * 
                        ////////////////////////////////////////////////////////////////////////////////////////////*/
                        bufSingleTransfer[nLocalCount].Free();
                        bufDataAllocation[nLocalCount].Free();
                        bufPktsInfo[nLocalCount].Free();
                        handleOverlap[nLocalCount].Free();

                        cBufs[nLocalCount] = null;
                        xBufs[nLocalCount] = null;
                        oLaps[nLocalCount] = null;
                    }
                }
                GC.Collect();

                usb_setting(3);

                EndPointListSelectIdx = 1;
                EndPoint.TimeOut = 500;

                bool bResult = true;
                int xferLen = 4096;
                byte[] inData = new byte[xferLen];
                while (bResult)
                {
                    bResult = EndPoint.XferData(ref inData, ref xferLen);
                }
                EndPoint.TimeOut = 500;

                Trace.WriteLine("HY : [Try] send setting value");
                // 3. 셋팅값 전송
                usb_setting(1);

                // 4. 데이터 Read
                EndPointListSelectIdx = 1;

                BufSz = EndPoint.MaxPktSize * PpxInfo;
                p_error = EndPoint.MaxPktSize; // 16384
                QueueSz = QueueInfo; // 값 1로 고정
                PPX = PpxInfo;

                EndPoint.XferSize = BufSz;

                if (EndPoint is CyIsocEndPoint)
                    IsoPktBlockSize = (EndPoint as CyIsocEndPoint).GetPktBlockSize(BufSz);
                else
                    IsoPktBlockSize = 0;






                IsListening = true;
                FlagFinalCall = 0;
                Debug.WriteLine("HY : [Try] Start XferThread");
                //ListenUSBThread = new Thread(new ThreadStart( XferThread));
                //ListenUSBThread.Start();
                ListenUBSTask = Task.Run(() => XferThread());

                return;
            }            

            Debug.WriteLine("XferData Done!!!");
            unsafe
            {
                //Trace.WriteLine("HY : [Try] TryTake loop(test_buffer reset)");
                while (true)
                {
                    int testBuffcount = DataInQueue.Count;
                    if (testBuffcount != 0)
                    {
                        Thread.Sleep(0);
                        //Debug.WriteLine("Remaining test_buffer count is " + testBuffcount);
                    }
                    else
                        break;
                }

                for (nLocalCount = 0; nLocalCount < QueueSz; nLocalCount++)
                {
                    CyUSB.OVERLAPPED ovLapStatus = new CyUSB.OVERLAPPED();
                    ovLapStatus = (CyUSB.OVERLAPPED)Marshal.PtrToStructure(handleOverlap[nLocalCount].AddrOfPinnedObject(), typeof(CyUSB.OVERLAPPED));
                    PInvoke.CloseHandle(ovLapStatus.hEvent);

                    /*////////////////////////////////////////////////////////////////////////////////////////////
                     * 
                     * Release the pinned allocation handles.
                     * 
                    ////////////////////////////////////////////////////////////////////////////////////////////*/
                    bufSingleTransfer[nLocalCount].Free();
                    bufDataAllocation[nLocalCount].Free();
                    bufPktsInfo[nLocalCount].Free();
                    handleOverlap[nLocalCount].Free();

                    cBufs[nLocalCount] = null;
                    xBufs[nLocalCount] = null;
                    oLaps[nLocalCount] = null;
                }
            }
            GC.Collect();
        }

        /// <summary>
        ///[함수] USB 관련 함수 | Called at the end of recursive method, LockNLoad().
        ///                       XferData() implements the infinite transfer loop
        /// </summary>
        /// <param name="cBufs"></param>
        /// <param name="xBufs"></param>
        /// <param name="oLaps"></param>
        /// <param name="pktsInfo"></param>
        /// <param name="handleOverlap"></param>
        public unsafe void XferData(byte[][] cBufs, byte[][] xBufs, byte[][] oLaps, ISO_PKT_INFO[][] pktsInfo, GCHandle[] handleOverlap)
        {
            int k = 0;
            int len = 0;
            int pre_successes = 0;

            Successes = 0;
            Failures = 0;

            XferBytes = 0;
            t1 = DateTime.Now;
            long nIteration = 0;
            CyUSB.OVERLAPPED ovData = new CyUSB.OVERLAPPED();
            // 사장님 - USB Receive Loop
            while (IsListening)
            {
                nIteration++;
                // WaitForXfer
                unsafe
                {
                    //fixed (byte* tmpOvlap = oLaps[k])
                    {
                        ovData = (CyUSB.OVERLAPPED)Marshal.PtrToStructure(handleOverlap[k].AddrOfPinnedObject(), typeof(CyUSB.OVERLAPPED));
                        if (!EndPoint.WaitForXfer(ovData.hEvent, 1000))
                        {
                            EndPoint.Abort();
                            PInvoke.WaitForSingleObject(ovData.hEvent, 0);

                            EndPoint.FinishDataXfer(ref cBufs[k], ref xBufs[k], ref len, ref oLaps[k]);
                            
                            for (int i = 0; i < 512; ++i) // 16384 * 16
                            {
                                int check_write = 0;
                                for (int ii = 0; ii < 16384; ++ii)
                                {
                                    if (xBufs[k][i * 16384 + ii] != 0xa5)
                                    {
                                        check_write = 1;
                                        break;
                                    }
                                }
                                if (check_write == 0)
                                    break;
                                byte[] temp_buffer = new byte[16384];
                                for (int ii = 0; ii < 16384; ++ii)
                                {
                                    temp_buffer[ii] = xBufs[k][i * 16384 + ii];
                                }
                                //Buffer.BlockCopy(xBufs[k], 0, temp_buffer, 0, 16384);
                                //while (DataInQueue.Post(temp_buffer) == false)
                                DataInQueue.Add(temp_buffer);
                                // 넣을때
                                for (int ii = 0; ii < 16384; ++ii)
                                {
                                    xBufs[k][i * 16384 + ii] = DefaultBufInitValue;
                                }
                                XferBytes += 16384;
                                test_data += 16384;
                                //test_data2 = test_buffer.Count();
                                Successes++;
                            }
                        }
                        else
                        {
                            // FinishDataXfer
                            if (EndPoint.FinishDataXfer(ref cBufs[k], ref xBufs[k], ref len, ref oLaps[k]))
                            {
                                byte[] temp_buffer = new byte[len];
                                Buffer.BlockCopy(xBufs[k], 0, temp_buffer, 0, len);
                                //while (DataInQueue.Post(temp_buffer) == false) ;
                                DataInQueue.Add(temp_buffer);
                                // 넣을때

                                XferBytes += len;
                                test_data += len;
                                //test_data2 = test_buffer.Count();
                                Successes++;

                                for (int i = 0; i < xBufs[k].Length; i++)
                                    xBufs[k][i] = DefaultBufInitValue;
                            }
                            else
                            {
                                Trace.WriteLine("Fail");
                                Failures++;// Failures++;
                            }
                        }
                    }
                }

                if (FlagFinalCall == 2)
                {

                    usb_setting(2); // stop 눌렸을때 Final call
                    pre_successes = Successes;
                    FlagFinalCall = 1;
                }
                else if (FlagFinalCall == 1)
                {

                    if (pre_successes + 3 < Successes)
                        IsListening = false;
                }

                // Re-submit this buffer into the queue
                len = BufSz;
                if (EndPoint.BeginDataXfer(ref cBufs[k], ref xBufs[k], ref len, ref oLaps[k]) == false)
                {
                    Failures++;// Failures++;
                }
                k++;
                if (k == QueueSz)  // Only update displayed stats once each time through the queue
                {
                    k = 0;

                    t2 = DateTime.Now;
                    elapsed = t2 - t1;

                    xferRate = (long)(XferBytes / elapsed.TotalMilliseconds);
                    xferRate = xferRate / (int)100 * (int)100;

                    // Call StatusUpdate() in the main thread
                    //                    if (bRunning == true) this.Invoke(updateUI);


                    // For small QueueSz or PPX, the loop is too tight for UI thread to ever get service.   
                    // Without this, app hangs in those scenarios.
                    
                }
                
                if (Failures > 100)
                {
                    EndPoint.Abort();
                    throw new CyUSBBufferFailException();                    
                }
                

            } // End infinite loop
            // Let's recall all the queued buffer and abort the end point.
            EndPoint.Abort();
        }


        #endregion
    }

    public class CyUSBBufferFailException : Exception
    {
        public CyUSBBufferFailException()
        {
        }

        public CyUSBBufferFailException(string message)
            : base(message)
        {
        }

        public CyUSBBufferFailException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }
}

