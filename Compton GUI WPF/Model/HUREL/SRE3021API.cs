using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace HUREL.Compton.CZT
{
    #region Packet classes


    /// <summary>
    /// Packet Header. Total 10 bytes
    /// </summary>
    internal class SRE3021PacketHeader
    {
        /// <summary>
        /// 3 bits. default 0
        /// </summary>
        public int Version { get; init; }
        /// <summary>
        /// 5 bits
        /// </summary>
        public int SystemNumber { get; init; }
        /// <summary>
        /// 8 bits, PacketType
        /// </summary>
        public SRE3021PacketType PacketType { get; init; }
        /// <summary>
        /// 2 bits, PacketSeqeunce
        /// </summary>
        public SRE3021PacketSequence SequenceFlag { get; init; }
        /// <summary>
        /// 14 bits, Incremental counter for packet identification
        /// </summary>
        public int PacketCount { get; init; }
        /// <summary>
        /// 32 bits, fixed = 0 // or TimeStamp
        /// </summary>
        public int Reserved { get; init; }
        /// <summary>
        /// Uint16, Data length
        /// </summary>
        public int DataLength { get; init; }

        /// <summary>
        /// Byte array, Decode / Encode data
        /// </summary>
        public byte[] ByteData { get; init; }

        /// <summary>
        /// Type
        /// </summary>
        public bool IsFromPCToSys { get; init; }
        public SRE3021PacketHeader(byte[] byteArray)
        {
            ByteData = new byte[10];
            byteArray.CopyTo(ByteData, 0);

            (Version, SystemNumber, PacketType) = CalcPacketID(new byte[] { ByteData[1], ByteData[0] });
            (SequenceFlag, PacketCount) = CalcPacketSequence(new byte[] { ByteData[3], ByteData[2] });
            Reserved = 0;
            DataLength = GetValueFromBytes(new byte[] { ByteData[9], ByteData[8] }, 0, 15);

            IsFromPCToSys = false;
        }

        private (int version, int systemNum, SRE3021PacketType packetType) CalcPacketID(byte[] data)
        {
            Debug.Assert(data.Length == 2, "CalcPacketID failed, Data length is must 2");
            int ver = GetValueFromBytes(new byte[] { data[1] }, 5, 7);
            int sysNum = GetValueFromBytes(new byte[] { data[1] }, 0, 4);
            SRE3021PacketType pType = (SRE3021PacketType)GetValueFromBytes(new byte[] { data[0] }, 0, 7);

            return (ver, sysNum, pType);
        }

        private (SRE3021PacketSequence sequenceFlag, int packetCount) CalcPacketSequence(byte[] data)
        {
            Debug.Assert(data.Length == 2, "CalcPacketSequence failed, Data length is must 2");
            int seFlag = GetValueFromBytes(new byte[] { data[1] }, 6, 7);
            int pCount = GetValueFromBytes(new byte[] { data[0], data[1] }, 0, 4);

            return ((SRE3021PacketSequence)seFlag, pCount);
        }

        private int GetValueFromBytes(byte[] bData, uint startIdx, uint endIdx)
        {
            Debug.Assert(startIdx < endIdx);
            BitArray bits = new BitArray(bData);
            BitArray result = new BitArray((int)(endIdx - startIdx + 1));

            for (int i = (int)startIdx; i <= endIdx; ++i)
            {
                result[i - (int)startIdx] = bits[i];
            }

            Debug.Assert(result.Length <= 32);

            int[] array = new int[1];
            result.CopyTo(array, 0);
            return array[0];
        }

        public SRE3021PacketHeader(SRE3021PacketType packetType, UInt16 datalength)
        {
            Version = 0;
            SystemNumber = 0;
            PacketType = packetType;
            SequenceFlag = 0;
            PacketCount = TotalPacketCount;
            Reserved = 0;
            DataLength = datalength;
            IsFromPCToSys = true;
            byte resultPacketID_SysNum = 0x00;
            byte resultPacket_Type = (byte)packetType;

            var resultPacketSequence = BitConverter.GetBytes((UInt16)TotalPacketCount);


            var resultPacketTimestamp = new byte[4] { 0x0, 0x00, 0x00, 0x00 };

            var resultPacketDatalength = BitConverter.GetBytes(datalength);

            ByteData = new byte[10] { resultPacketID_SysNum, resultPacket_Type, resultPacketSequence[1], resultPacketSequence[0], resultPacketTimestamp[3], resultPacketTimestamp[2], resultPacketTimestamp[1], resultPacketTimestamp[0], resultPacketDatalength[1], resultPacketDatalength[0] };
            ++TotalPacketCount;
            int maxTotalPacketCount = 0b0100_0000_0000_0000 - 1;
            if (TotalPacketCount >= maxTotalPacketCount)
            {
                TotalPacketCount = 0;
            }
        }

        static int TotalPacketCount = 0;


    }

    /// <summary>
    /// The “Packet Type” field of the Packet Header defines how the Packet Data field shall be decoded.
    /// </summary>
    public enum SRE3021PacketType
    {
        /// <summary>
        /// Unused Packet type for the SRE3021
        /// </summary>
        UNUSED = 0,
        /// <summary>
        /// Writes to a system register
        /// </summary>
        WRITE_SYS_REG = 0x10,
        /// <summary>
        /// Reads a system register 
        /// </summary>
        READ_SYS_REG = 0x11,
        /// <summary>
        /// Read-back packet to be sent after a “Write System Register” or “Read System Register”
        /// </summary>
        READBACK_SYS_REG = 0x12,
        /// <summary>
        /// Write/Read to an ASIC configuration register
        /// </summary>
        RW_ASIC_REG = 0xC0,
        /// <summary>
        /// Read-back packet to be sent after a “ASIC configuration register write/read”
        /// </summary>
        READBACK_ASIC_REG = 0xC1,
        /// <summary>
        /// Image data format from the SRE3021
        /// </summary>
        IMG_DATA = 0xD1
    }

    /// <summary>
    /// specifies what sequence (e.g. for read-out data, there will be a series of packets all in the same sequence) the packet is a part of:
    /// </summary>
    public enum SRE3021PacketSequence
    {
        /// <summary>
        /// packet is the only packet in the sequence.
        /// </summary>
        STAND_ALONE = 0,
        /// <summary>
        /// packet is the first packet in a sequence.
        /// </summary>
        FIRST_PACKET = 1,
        /// <summary>
        /// packet continues a started sequence.
        /// </summary>
        COUNTINUATION_PACKET = 2,
        /// <summary>
        /// packet is the last packet in the sequence.
        /// </summary>
        LAST_PACKET = 3
    }

    /// <summary>
    /// Enum of system register's address
    /// </summary>
    public enum SRE3021SysRegisterADDR
    {
        /// <summary>
        /// R, 4, Unique serial number of unit.
        /// </summary>
        SERIAL_NUMBER = 0x0000,
        /// <summary>
        /// R, 2,  Type of firmware
        /// </summary>
        FIRMWARE_TYPE = 0X0001,
        /// <summary>
        /// R, 2, Firmware version number.
        /// </summary>
        FIRMWARE_VERSION = 0X0002,
        /// <summary>
        /// R/W, 1, System ID
        /// </summary>
        SYSTEM_NUMBER = 0X0010,
        /// <summary>
        /// R/W, 2, The current value of the readout packet counter.
        /// </summary>
        //READOUT_PACKET_COUNTER = 0X0011,
        /// <summary>
        /// R, 2, Soc temperature?
        /// </summary>
        SOC_TEMP = 0X0900,
        /// <summary>
        /// R, 2, Max soc temperature?
        /// </summary>
        SOC_TEMP_MAX = 0X0901,
        /// <summary>
        /// R, 2, Min soc temperature?
        /// </summary>
        SOC_TEMP_MIN = 0X0902,
        /// <summary>
        /// R, 2, Analog VDD
        /// </summary>
        AVDD = 0X0A00,
        /// <summary>
        /// R, 2, Analog VSS
        /// </summary>
        AVSS = 0X0A01,
        /// <summary>
        /// R, 2, Dectector voltage?
        /// </summary>
        DETECTOR_V = 0X0A02,
        /// <summary>
        /// R, 2, Dectector current?
        /// </summary>
        DETECTOR_A = 0X0A03,
        /// <summary>
        /// R, 2, Systme temperature?
        /// </summary>
        //SYS_TEMP = 0X0A04,

        /// <summary>
        /// P, 1, Start calibration pulsing with current settings. 
        /// </summary>
        CAL_EXECUTE = 0X0C00,
        /// <summary>
        /// R/W, 1, Falling/rising edges of cal pulse. 0 = falling
        /// </summary>
        CAL_PULSE_POLAR = 0X0C01,
        /// <summary>
        /// R/W, 2, Number of pulses to generate.
        /// </summary>
        CAL_NUM_PULSE = 0X0C02,
        /// <summary>
        /// R/W, 4, Length of cal pulses in number of system clock cycles. Actual length is +1
        /// </summary>
        CAL_PULSE_LENGTH = 0X0C03,
        /// <summary>
        /// R/W, 4, Clock cycles between start of each cal pulse. Actual length is +1.
        /// </summary>
        CAL_PULSE_INTERVAL = 0X0C04,

        /// <summary>
        /// R/W, 2, Threshold normal channels
        /// </summary>
        VTHR = 0X0E00,
        /// <summary>
        /// R/W, 2, Threshold special channels
        /// </summary>
        VTHR0 = 0X0E01,
        /// <summary>
        /// R/W, 2, Preamp feedback resistance normal channels
        /// </summary>
        VFP = 0X0E02,
        /// <summary>
        /// R/W, 2, Preamp feedback resistance special channels
        /// </summary>
        VFP0 = 0X0E03,
        /// <summary>
        /// R/W, 2, Detector Grid bias, T.B.D – Setting this value to max will set the detector anode guard pin to 0V
        /// </summary>
        GRID_DAC = 0X0E04,
        /// <summary>
        /// R/W, 2, Detector HV bias 100DAC step=-100V
        /// </summary>
        HV_DAC = 0X0E05,
        /// <summary>
        /// R/W, 2, VRCControl voltage for Highpass filter resistor (NMOS) in front of discriminator (normal channel). Internally generated.
        /// </summary>
        VRC_DAC = 0X0E06,
        /// <summary>
        /// R/W, 2, DAC for cal pulse height. Sets the amplitude of the pulse used in the calibration circuit
        /// </summary>
        CAL_DAC = 0X0E07,

        /// <summary>
        /// R/W, 2, Number of channels in VATA chain
        /// </summary>
        CFG_NUM_CHANNEL = 0XF009,
        /// <summary>
        /// R/W 2, 
        /// </summary>
        CFG_HOLD_DLY = 0XF00A,
        /// <summary>
        /// R/W 2,
        /// </summary>
        CFG_SETTL_DLY = 0XF00B,
        /// <summary>
        /// R/W 2,
        /// </summary>
        CFG_T_CK_LOW = 0XF00C,
        /// <summary>
        /// R/W 2,
        /// </summary>
        CFG_T_CK_HIGH = 0XF00D,
        /// <summary>
        /// R/W 2,
        /// </summary>
        CFG_AFTER_READ_DLY = 0XF00E,
        /// <summary>
        /// R/W 2,
        /// </summary>
        CFG_SIG_CTRL = 0XF00F,
        /// <summary>
        /// addr_cfg_fixed_en
        /// </summary>
        CMD_SET_CH = 0XF010,
        /// <summary>
        /// R/W 1,
        /// </summary>
        CFG_PHYSTRIG_EN = 0XF011,
        /// <summary>
        /// R/W 1,
        /// </summary>
        CFG_CALTRIG_EN = 0XF012,
        /// <summary>
        /// R/W 1,
        /// </summary>
        CFG_FIXED_CH = 0XF013
    }
    public enum RWType
    {
        R,
        RW,
        P
    }

    /// <summary>
    /// SRE3021 Sysreg API Address
    /// </summary>
    public class SRE3021SysReg
    {
        public SRE3021SysRegisterADDR Address { get; init; }
        public string AddressName
        {
            get
            {
                return Address.ToString();
            }
        }
        public uint BitLength { get; init; }
        public RWType Type { get; init; }
        public long Value { get; set; }
        public SRE3021SysReg(SRE3021SysRegisterADDR addr, uint bitLength)
        {
            Address = addr;
            BitLength = bitLength;
            Type = CheckType(addr);
        }

        internal static RWType CheckType(SRE3021SysRegisterADDR addr)
        {
            UInt16 shortA = (UInt16)addr;
            if (shortA <= 0x0002)
            {
                return RWType.R;
            }
            else if (shortA > 0x0002 && shortA <= 0x11)
            {
                return RWType.RW;
            }
            else if (shortA >= 0x0900 && shortA <= 0x0A03)
            {
                return RWType.R;
            }
            else if (shortA == 0x0C00)
            {
                return RWType.P;
            }
            else if (shortA >= 0x0C01 && shortA <= 0xF00F)
            {
                return RWType.RW;
            }
            else if (shortA == 0xF010)
            {
                return RWType.P;
            }
            else if (shortA >= 0XF011 && shortA <= 0xf013)
            {
                return RWType.RW;
            }
            else
            {
                return RWType.R;
            }
        }

        internal static int GetByteLength(SRE3021SysRegisterADDR addr)
        {
            UInt16 shortA = (UInt16)addr;
            if (shortA == 0x0000)
            {
                return 4;
            }
            else if (shortA >= 0x0001 && shortA <= 0x0002)
            {
                return 2;
            }
            else if (shortA == 0x0010)
            {
                return 1;
            }
            else if (shortA >= 0x0900 && shortA <= 0x0A03)
            {
                return 2;
            }
            else if (shortA >= 0x0C00 && shortA <= 0x0C01)
            {
                return 1;
            }
            else if (shortA == 0x0C02)
            {
                return 2;
            }
            else if (shortA >= 0x0C03 && shortA <= 0x0C04)
            {
                return 4;
            }
            else if (shortA >= 0x0E00 && shortA <= 0xF00F)
            {
                return 2;
            }
            else if (shortA == 0xF010)
            {
                return 1;
            }
            else if (shortA >= 0XF011 && shortA <= 0xf013)
            {
                return 1;
            }
            else
            {
                throw new ArgumentException();
            }
        }
    }

    public enum SRE3021ASICRegisterADDR
    {
        NOTSELECT = 0,
        Disable_discriminator = 1,
        Channel_0_monitor_enable = 2,
        Channel_1_monitor_enable = 3,
        Channel_2_monitor_enable = 4,
        Channel_3_monitor_enable = 5,
        Channel_4_monitor_enable = 6,
        Channel_04_monitor_output_enable = 7,
        Current_compensation_enable = 8,
        Test_on = 9,
        sel = 10,
        Not_in_use_Cat_0 = 11,
        Not_in_use_Cat_1 = 12,
        Cathode_Channel_Disable = 13,
        Not_in_use_Cat_2 = 14,
        Not_in_use_Cat_3 = 15,
        Anode_Channe_0_Disable = 16,
        Anode_Channel_1_Disable = 17,
        Anode_Channel_2_Disable = 18,
        Anode_Channel_3_Disable = 19,
        Anode_Channel_4_Disable = 20,
        Anode_Channel_5_Disable = 21,
        Anode_Channel_6_Disable = 22,
        Anode_Channel_7_Disable = 23,
        Anode_Channel_8_Disable = 24,
        Anode_Channel_9_Disable = 25,
        Anode_Channel_10_Disable = 26,
        Anode_Channel_11_Disable = 27,
        Anode_Channel_12_Disable = 28,
        Anode_Channel_13_Disable = 29,
        Anode_Channel_14_Disable = 30,
        Anode_Channel_15_Disable = 31,
        Anode_Channel_16_Disable = 32,
        Anode_Channel_17_Disable = 33,
        Anode_Channel_18_Disable = 34,
        Anode_Channel_19_Disable = 35,
        Anode_Channel_20_Disable = 36,
        Anode_Channel_21_Disable = 37,
        Anode_Channel_22_Disable = 38,
        Anode_Channel_23_Disable = 39,
        Anode_Channel_24_Disable = 40,
        Anode_Channel_25_Disable = 41,
        Anode_Channel_26_Disable = 42,
        Anode_Channel_27_Disable = 43,
        Anode_Channel_28_Disable = 44,
        Not_in_use_in34 = 45,
        Anode_Channel_29_Disable = 46,
        Anode_Channel_30_Disable = 47,
        Anode_Channel_31_Disable = 48,
        Not_in_use_grid = 49,
        Anode_Channel_32_Disable = 50,
        Anode_Channel_33_Disable = 51,
        Anode_Channel_34_Disable = 52,
        Anode_Channel_35_Disable = 53,
        Anode_Channel_36_Disable = 54,
        Anode_Channel_37_Disable = 55,
        Anode_Channel_38_Disable = 56,
        Anode_Channel_39_Disable = 57,
        Anode_Channel_40_Disable = 58,
        Anode_Channel_41_Disable = 59,
        Anode_Channel_42_Disable = 60,
        Anode_Channel_43_Disable = 61,
        Anode_Channel_44_Disable = 62,
        Anode_Channel_45_Disable = 63,
        Anode_Channel_46_Disable = 64,
        Anode_Channel_47_Disable = 65,
        Anode_Channel_48_Disable = 66,
        Anode_Channel_49_Disable = 67,
        Anode_Channel_50_Disable = 68,
        Anode_Channel_51_Disable = 69,
        Anode_Channel_52_Disable = 70,
        Anode_Channel_53_Disable = 71,
        Anode_Channel_54_Disable = 72,
        Anode_Channel_55_Disable = 73,
        Anode_Channel_56_Disable = 74,
        Anode_Channel_57_Disable = 75,
        Anode_Channel_58_Disable = 76,
        Anode_Channel_59_Disable = 77,
        Anode_Channel_60_Disable = 78,
        Anode_Channel_61_Disable = 79,
        Anode_Channel_62_Disable = 80,
        Anode_Channel_63_Disable = 81,
        Anode_Channel_64_Disable = 82,
        Anode_Channel_65_Disable = 83,
        Anode_Channel_66_Disable = 84,
        Anode_Channel_67_Disable = 85,
        Anode_Channel_68_Disable = 86,
        Anode_Channel_69_Disable = 87,
        Anode_Channel_70_Disable = 88,
        Anode_Channel_71_Disable = 89,
        Anode_Channel_72_Disable = 90,
        Anode_Channel_73_Disable = 91,
        Anode_Channel_74_Disable = 92,
        Anode_Channel_75_Disable = 93,
        Anode_Channel_76_Disable = 94,
        Anode_Channel_77_Disable = 95,
        Anode_Channel_78_Disable = 96,
        Anode_Channel_79_Disable = 97,
        Anode_Channel_80_Disable = 98,
        Anode_Channel_81_Disable = 99,
        Anode_Channel_82_Disable = 100,
        Anode_Channel_83_Disable = 101,
        Anode_Channel_84_Disable = 102,
        Anode_Channel_85_Disable = 103,
        Anode_Channel_86_Disable = 104,
        Anode_Channel_87_Disable = 105,
        Anode_Channel_88_Disable = 106,
        Anode_Channel_89_Disable = 107,
        Anode_Channel_90_Disable = 108,
        Anode_Channel_91_Disable = 109,
        Anode_Channel_92_Disable = 110,
        Anode_Channel_93_Disable = 111,
        Anode_Channel_94_Disable = 112,
        Anode_Channel_95_Disable = 113,
        Anode_Channel_96_Disable = 114,
        Anode_Channel_97_Disable = 115,
        Anode_Channel_98_Disable = 116,
        Anode_Channel_99_Disable = 117,
        Anode_Channel_100_Disable = 118,
        Anode_Channel_101_Disable = 119,
        Anode_Channel_102_Disable = 120,
        Anode_Channel_103_Disable = 121,
        Anode_Channel_104_Disable = 122,
        Anode_Channel_105_Disable = 123,
        Anode_Channel_106_Disable = 124,
        Anode_Channel_107_Disable = 125,
        Anode_Channel_108_Disable = 126,
        Anode_Channel_109_Disable = 127,
        Anode_Channel_110_Disable = 128,
        Anode_Channel_111_Disable = 129,
        Anode_Channel_112_Disable = 130,
        Anode_Channel_113_Disable = 131,
        Anode_Channel_114_Disable = 132,
        Anode_Channel_115_Disable = 133,
        Anode_Channel_116_Disable = 134,
        Anode_Channel_117_Disable = 135,
        Anode_Channel_118_Disable = 136,
        Anode_Channel_119_Disable = 137,
        Anode_Channel_120_Disable = 138,
        THRESHOLD_TRIM_DAC_0_0 = 139,
        THRESHOLD_TRIM_0_DAC_0 = 139,
        THRESHOLD_TRIM_0_DAC_1 = 140,
        THRESHOLD_TRIM_0_DAC_2 = 141,
        THRESHOLD_TRIM_0_DAC_3 = 142,
        THRESHOLD_TRIM_1_DAC_0 = 143,
        THRESHOLD_TRIM_1_DAC_1 = 144,
        THRESHOLD_TRIM_1_DAC_2 = 145,
        THRESHOLD_TRIM_1_DAC_3 = 146,
        THRESHOLD_TRIM_2_DAC_0 = 147,
        THRESHOLD_TRIM_2_DAC_1 = 148,
        THRESHOLD_TRIM_2_DAC_2 = 149,
        THRESHOLD_TRIM_2_DAC_3 = 150,
        THRESHOLD_TRIM_3_DAC_0 = 151,
        THRESHOLD_TRIM_3_DAC_1 = 152,
        THRESHOLD_TRIM_3_DAC_2 = 153,
        THRESHOLD_TRIM_3_DAC_3 = 154,
        THRESHOLD_TRIM_4_DAC_0 = 155,
        THRESHOLD_TRIM_4_DAC_1 = 156,
        THRESHOLD_TRIM_4_DAC_2 = 157,
        THRESHOLD_TRIM_4_DAC_3 = 158,
        THRESHOLD_TRIM_5_DAC_0 = 159,
        THRESHOLD_TRIM_5_DAC_1 = 160,
        THRESHOLD_TRIM_5_DAC_2 = 161,
        THRESHOLD_TRIM_5_DAC_3 = 162,
        THRESHOLD_TRIM_6_DAC_0 = 163,
        THRESHOLD_TRIM_6_DAC_1 = 164,
        THRESHOLD_TRIM_6_DAC_2 = 165,
        THRESHOLD_TRIM_6_DAC_3 = 166,
        THRESHOLD_TRIM_7_DAC_0 = 167,
        THRESHOLD_TRIM_7_DAC_1 = 168,
        THRESHOLD_TRIM_7_DAC_2 = 169,
        THRESHOLD_TRIM_7_DAC_3 = 170,
        THRESHOLD_TRIM_8_DAC_0 = 171,
        THRESHOLD_TRIM_8_DAC_1 = 172,
        THRESHOLD_TRIM_8_DAC_2 = 173,
        THRESHOLD_TRIM_8_DAC_3 = 174,
        THRESHOLD_TRIM_9_DAC_0 = 175,
        THRESHOLD_TRIM_9_DAC_1 = 176,
        THRESHOLD_TRIM_9_DAC_2 = 177,
        THRESHOLD_TRIM_9_DAC_3 = 178,
        THRESHOLD_TRIM_10_DAC_0 = 179,
        THRESHOLD_TRIM_10_DAC_1 = 180,
        THRESHOLD_TRIM_10_DAC_2 = 181,
        THRESHOLD_TRIM_10_DAC_3 = 182,
        THRESHOLD_TRIM_11_DAC_0 = 183,
        THRESHOLD_TRIM_11_DAC_1 = 184,
        THRESHOLD_TRIM_11_DAC_2 = 185,
        THRESHOLD_TRIM_11_DAC_3 = 186,
        THRESHOLD_TRIM_12_DAC_0 = 187,
        THRESHOLD_TRIM_12_DAC_1 = 188,
        THRESHOLD_TRIM_12_DAC_2 = 189,
        THRESHOLD_TRIM_12_DAC_3 = 190,
        THRESHOLD_TRIM_13_DAC_0 = 191,
        THRESHOLD_TRIM_13_DAC_1 = 192,
        THRESHOLD_TRIM_13_DAC_2 = 193,
        THRESHOLD_TRIM_13_DAC_3 = 194,
        THRESHOLD_TRIM_14_DAC_0 = 195,
        THRESHOLD_TRIM_14_DAC_1 = 196,
        THRESHOLD_TRIM_14_DAC_2 = 197,
        THRESHOLD_TRIM_14_DAC_3 = 198,
        THRESHOLD_TRIM_15_DAC_0 = 199,
        THRESHOLD_TRIM_15_DAC_1 = 200,
        THRESHOLD_TRIM_15_DAC_2 = 201,
        THRESHOLD_TRIM_15_DAC_3 = 202,
        THRESHOLD_TRIM_16_DAC_0 = 203,
        THRESHOLD_TRIM_16_DAC_1 = 204,
        THRESHOLD_TRIM_16_DAC_2 = 205,
        THRESHOLD_TRIM_16_DAC_3 = 206,
        THRESHOLD_TRIM_17_DAC_0 = 207,
        THRESHOLD_TRIM_17_DAC_1 = 208,
        THRESHOLD_TRIM_17_DAC_2 = 209,
        THRESHOLD_TRIM_17_DAC_3 = 210,
        THRESHOLD_TRIM_18_DAC_0 = 211,
        THRESHOLD_TRIM_18_DAC_1 = 212,
        THRESHOLD_TRIM_18_DAC_2 = 213,
        THRESHOLD_TRIM_18_DAC_3 = 214,
        THRESHOLD_TRIM_19_DAC_0 = 215,
        THRESHOLD_TRIM_19_DAC_1 = 216,
        THRESHOLD_TRIM_19_DAC_2 = 217,
        THRESHOLD_TRIM_19_DAC_3 = 218,
        THRESHOLD_TRIM_20_DAC_0 = 219,
        THRESHOLD_TRIM_20_DAC_1 = 220,
        THRESHOLD_TRIM_20_DAC_2 = 221,
        THRESHOLD_TRIM_20_DAC_3 = 222,
        THRESHOLD_TRIM_21_DAC_0 = 223,
        THRESHOLD_TRIM_21_DAC_1 = 224,
        THRESHOLD_TRIM_21_DAC_2 = 225,
        THRESHOLD_TRIM_21_DAC_3 = 226,
        THRESHOLD_TRIM_22_DAC_0 = 227,
        THRESHOLD_TRIM_22_DAC_1 = 228,
        THRESHOLD_TRIM_22_DAC_2 = 229,
        THRESHOLD_TRIM_22_DAC_3 = 230,
        THRESHOLD_TRIM_23_DAC_0 = 231,
        THRESHOLD_TRIM_23_DAC_1 = 232,
        THRESHOLD_TRIM_23_DAC_2 = 233,
        THRESHOLD_TRIM_23_DAC_3 = 234,
        THRESHOLD_TRIM_24_DAC_0 = 235,
        THRESHOLD_TRIM_24_DAC_1 = 236,
        THRESHOLD_TRIM_24_DAC_2 = 237,
        THRESHOLD_TRIM_24_DAC_3 = 238,
        THRESHOLD_TRIM_25_DAC_0 = 239,
        THRESHOLD_TRIM_25_DAC_1 = 240,
        THRESHOLD_TRIM_25_DAC_2 = 241,
        THRESHOLD_TRIM_25_DAC_3 = 242,
        THRESHOLD_TRIM_26_DAC_0 = 243,
        THRESHOLD_TRIM_26_DAC_1 = 244,
        THRESHOLD_TRIM_26_DAC_2 = 245,
        THRESHOLD_TRIM_26_DAC_3 = 246,
        THRESHOLD_TRIM_27_DAC_0 = 247,
        THRESHOLD_TRIM_27_DAC_1 = 248,
        THRESHOLD_TRIM_27_DAC_2 = 249,
        THRESHOLD_TRIM_27_DAC_3 = 250,
        THRESHOLD_TRIM_28_DAC_0 = 251,
        THRESHOLD_TRIM_28_DAC_1 = 252,
        THRESHOLD_TRIM_28_DAC_2 = 253,
        THRESHOLD_TRIM_28_DAC_3 = 254,
        THRESHOLD_TRIM_29_DAC_0 = 255,
        THRESHOLD_TRIM_29_DAC_1 = 256,
        THRESHOLD_TRIM_29_DAC_2 = 257,
        THRESHOLD_TRIM_29_DAC_3 = 258,
        THRESHOLD_TRIM_30_DAC_0 = 259,
        THRESHOLD_TRIM_30_DAC_1 = 260,
        THRESHOLD_TRIM_30_DAC_2 = 261,
        THRESHOLD_TRIM_30_DAC_3 = 262,
        THRESHOLD_TRIM_31_DAC_0 = 263,
        THRESHOLD_TRIM_31_DAC_1 = 264,
        THRESHOLD_TRIM_31_DAC_2 = 265,
        THRESHOLD_TRIM_31_DAC_3 = 266,
        THRESHOLD_TRIM_32_DAC_0 = 267,
        THRESHOLD_TRIM_32_DAC_1 = 268,
        THRESHOLD_TRIM_32_DAC_2 = 269,
        THRESHOLD_TRIM_32_DAC_3 = 270,
        THRESHOLD_TRIM_33_DAC_0 = 271,
        THRESHOLD_TRIM_33_DAC_1 = 272,
        THRESHOLD_TRIM_33_DAC_2 = 273,
        THRESHOLD_TRIM_33_DAC_3 = 274,
        THRESHOLD_TRIM_34_DAC_0 = 275,
        THRESHOLD_TRIM_34_DAC_1 = 276,
        THRESHOLD_TRIM_34_DAC_2 = 277,
        THRESHOLD_TRIM_34_DAC_3 = 278,
        THRESHOLD_TRIM_35_DAC_0 = 279,
        THRESHOLD_TRIM_35_DAC_1 = 280,
        THRESHOLD_TRIM_35_DAC_2 = 281,
        THRESHOLD_TRIM_35_DAC_3 = 282,
        THRESHOLD_TRIM_36_DAC_0 = 283,
        THRESHOLD_TRIM_36_DAC_1 = 284,
        THRESHOLD_TRIM_36_DAC_2 = 285,
        THRESHOLD_TRIM_36_DAC_3 = 286,
        THRESHOLD_TRIM_37_DAC_0 = 287,
        THRESHOLD_TRIM_37_DAC_1 = 288,
        THRESHOLD_TRIM_37_DAC_2 = 289,
        THRESHOLD_TRIM_37_DAC_3 = 290,
        THRESHOLD_TRIM_38_DAC_0 = 291,
        THRESHOLD_TRIM_38_DAC_1 = 292,
        THRESHOLD_TRIM_38_DAC_2 = 293,
        THRESHOLD_TRIM_38_DAC_3 = 294,
        THRESHOLD_TRIM_39_DAC_0 = 295,
        THRESHOLD_TRIM_39_DAC_1 = 296,
        THRESHOLD_TRIM_39_DAC_2 = 297,
        THRESHOLD_TRIM_39_DAC_3 = 298,
        THRESHOLD_TRIM_40_DAC_0 = 299,
        THRESHOLD_TRIM_40_DAC_1 = 300,
        THRESHOLD_TRIM_40_DAC_2 = 301,
        THRESHOLD_TRIM_40_DAC_3 = 302,
        THRESHOLD_TRIM_41_DAC_0 = 303,
        THRESHOLD_TRIM_41_DAC_1 = 304,
        THRESHOLD_TRIM_41_DAC_2 = 305,
        THRESHOLD_TRIM_41_DAC_3 = 306,
        THRESHOLD_TRIM_42_DAC_0 = 307,
        THRESHOLD_TRIM_42_DAC_1 = 308,
        THRESHOLD_TRIM_42_DAC_2 = 309,
        THRESHOLD_TRIM_42_DAC_3 = 310,
        THRESHOLD_TRIM_43_DAC_0 = 311,
        THRESHOLD_TRIM_43_DAC_1 = 312,
        THRESHOLD_TRIM_43_DAC_2 = 313,
        THRESHOLD_TRIM_43_DAC_3 = 314,
        THRESHOLD_TRIM_44_DAC_0 = 315,
        THRESHOLD_TRIM_44_DAC_1 = 316,
        THRESHOLD_TRIM_44_DAC_2 = 317,
        THRESHOLD_TRIM_44_DAC_3 = 318,
        THRESHOLD_TRIM_45_DAC_0 = 319,
        THRESHOLD_TRIM_45_DAC_1 = 320,
        THRESHOLD_TRIM_45_DAC_2 = 321,
        THRESHOLD_TRIM_45_DAC_3 = 322,
        THRESHOLD_TRIM_46_DAC_0 = 323,
        THRESHOLD_TRIM_46_DAC_1 = 324,
        THRESHOLD_TRIM_46_DAC_2 = 325,
        THRESHOLD_TRIM_46_DAC_3 = 326,
        THRESHOLD_TRIM_47_DAC_0 = 327,
        THRESHOLD_TRIM_47_DAC_1 = 328,
        THRESHOLD_TRIM_47_DAC_2 = 329,
        THRESHOLD_TRIM_47_DAC_3 = 330,
        THRESHOLD_TRIM_48_DAC_0 = 331,
        THRESHOLD_TRIM_48_DAC_1 = 332,
        THRESHOLD_TRIM_48_DAC_2 = 333,
        THRESHOLD_TRIM_48_DAC_3 = 334,
        THRESHOLD_TRIM_49_DAC_0 = 335,
        THRESHOLD_TRIM_49_DAC_1 = 336,
        THRESHOLD_TRIM_49_DAC_2 = 337,
        THRESHOLD_TRIM_49_DAC_3 = 338,
        THRESHOLD_TRIM_50_DAC_0 = 339,
        THRESHOLD_TRIM_50_DAC_1 = 340,
        THRESHOLD_TRIM_50_DAC_2 = 341,
        THRESHOLD_TRIM_50_DAC_3 = 342,
        THRESHOLD_TRIM_51_DAC_0 = 343,
        THRESHOLD_TRIM_51_DAC_1 = 344,
        THRESHOLD_TRIM_51_DAC_2 = 345,
        THRESHOLD_TRIM_51_DAC_3 = 346,
        THRESHOLD_TRIM_52_DAC_0 = 347,
        THRESHOLD_TRIM_52_DAC_1 = 348,
        THRESHOLD_TRIM_52_DAC_2 = 349,
        THRESHOLD_TRIM_52_DAC_3 = 350,
        THRESHOLD_TRIM_53_DAC_0 = 351,
        THRESHOLD_TRIM_53_DAC_1 = 352,
        THRESHOLD_TRIM_53_DAC_2 = 353,
        THRESHOLD_TRIM_53_DAC_3 = 354,
        THRESHOLD_TRIM_54_DAC_0 = 355,
        THRESHOLD_TRIM_54_DAC_1 = 356,
        THRESHOLD_TRIM_54_DAC_2 = 357,
        THRESHOLD_TRIM_54_DAC_3 = 358,
        THRESHOLD_TRIM_55_DAC_0 = 359,
        THRESHOLD_TRIM_55_DAC_1 = 360,
        THRESHOLD_TRIM_55_DAC_2 = 361,
        THRESHOLD_TRIM_55_DAC_3 = 362,
        THRESHOLD_TRIM_56_DAC_0 = 363,
        THRESHOLD_TRIM_56_DAC_1 = 364,
        THRESHOLD_TRIM_56_DAC_2 = 365,
        THRESHOLD_TRIM_56_DAC_3 = 366,
        THRESHOLD_TRIM_57_DAC_0 = 367,
        THRESHOLD_TRIM_57_DAC_1 = 368,
        THRESHOLD_TRIM_57_DAC_2 = 369,
        THRESHOLD_TRIM_57_DAC_3 = 370,
        THRESHOLD_TRIM_58_DAC_0 = 371,
        THRESHOLD_TRIM_58_DAC_1 = 372,
        THRESHOLD_TRIM_58_DAC_2 = 373,
        THRESHOLD_TRIM_58_DAC_3 = 374,
        THRESHOLD_TRIM_59_DAC_0 = 375,
        THRESHOLD_TRIM_59_DAC_1 = 376,
        THRESHOLD_TRIM_59_DAC_2 = 377,
        THRESHOLD_TRIM_59_DAC_3 = 378,
        THRESHOLD_TRIM_60_DAC_0 = 379,
        THRESHOLD_TRIM_60_DAC_1 = 380,
        THRESHOLD_TRIM_60_DAC_2 = 381,
        THRESHOLD_TRIM_60_DAC_3 = 382,
        THRESHOLD_TRIM_61_DAC_0 = 383,
        THRESHOLD_TRIM_61_DAC_1 = 384,
        THRESHOLD_TRIM_61_DAC_2 = 385,
        THRESHOLD_TRIM_61_DAC_3 = 386,
        THRESHOLD_TRIM_62_DAC_0 = 387,
        THRESHOLD_TRIM_62_DAC_1 = 388,
        THRESHOLD_TRIM_62_DAC_2 = 389,
        THRESHOLD_TRIM_62_DAC_3 = 390,
        THRESHOLD_TRIM_63_DAC_0 = 391,
        THRESHOLD_TRIM_63_DAC_1 = 392,
        THRESHOLD_TRIM_63_DAC_2 = 393,
        THRESHOLD_TRIM_63_DAC_3 = 394,
        THRESHOLD_TRIM_64_DAC_0 = 395,
        THRESHOLD_TRIM_64_DAC_1 = 396,
        THRESHOLD_TRIM_64_DAC_2 = 397,
        THRESHOLD_TRIM_64_DAC_3 = 398,
        THRESHOLD_TRIM_65_DAC_0 = 399,
        THRESHOLD_TRIM_65_DAC_1 = 400,
        THRESHOLD_TRIM_65_DAC_2 = 401,
        THRESHOLD_TRIM_65_DAC_3 = 402,
        THRESHOLD_TRIM_66_DAC_0 = 403,
        THRESHOLD_TRIM_66_DAC_1 = 404,
        THRESHOLD_TRIM_66_DAC_2 = 405,
        THRESHOLD_TRIM_66_DAC_3 = 406,
        THRESHOLD_TRIM_67_DAC_0 = 407,
        THRESHOLD_TRIM_67_DAC_1 = 408,
        THRESHOLD_TRIM_67_DAC_2 = 409,
        THRESHOLD_TRIM_67_DAC_3 = 410,
        THRESHOLD_TRIM_68_DAC_0 = 411,
        THRESHOLD_TRIM_68_DAC_1 = 412,
        THRESHOLD_TRIM_68_DAC_2 = 413,
        THRESHOLD_TRIM_68_DAC_3 = 414,
        THRESHOLD_TRIM_69_DAC_0 = 415,
        THRESHOLD_TRIM_69_DAC_1 = 416,
        THRESHOLD_TRIM_69_DAC_2 = 417,
        THRESHOLD_TRIM_69_DAC_3 = 418,
        THRESHOLD_TRIM_70_DAC_0 = 419,
        THRESHOLD_TRIM_70_DAC_1 = 420,
        THRESHOLD_TRIM_70_DAC_2 = 421,
        THRESHOLD_TRIM_70_DAC_3 = 422,
        THRESHOLD_TRIM_71_DAC_0 = 423,
        THRESHOLD_TRIM_71_DAC_1 = 424,
        THRESHOLD_TRIM_71_DAC_2 = 425,
        THRESHOLD_TRIM_71_DAC_3 = 426,
        THRESHOLD_TRIM_72_DAC_0 = 427,
        THRESHOLD_TRIM_72_DAC_1 = 428,
        THRESHOLD_TRIM_72_DAC_2 = 429,
        THRESHOLD_TRIM_72_DAC_3 = 430,
        THRESHOLD_TRIM_73_DAC_0 = 431,
        THRESHOLD_TRIM_73_DAC_1 = 432,
        THRESHOLD_TRIM_73_DAC_2 = 433,
        THRESHOLD_TRIM_73_DAC_3 = 434,
        THRESHOLD_TRIM_74_DAC_0 = 435,
        THRESHOLD_TRIM_74_DAC_1 = 436,
        THRESHOLD_TRIM_74_DAC_2 = 437,
        THRESHOLD_TRIM_74_DAC_3 = 438,
        THRESHOLD_TRIM_75_DAC_0 = 439,
        THRESHOLD_TRIM_75_DAC_1 = 440,
        THRESHOLD_TRIM_75_DAC_2 = 441,
        THRESHOLD_TRIM_75_DAC_3 = 442,
        THRESHOLD_TRIM_76_DAC_0 = 443,
        THRESHOLD_TRIM_76_DAC_1 = 444,
        THRESHOLD_TRIM_76_DAC_2 = 445,
        THRESHOLD_TRIM_76_DAC_3 = 446,
        THRESHOLD_TRIM_77_DAC_0 = 447,
        THRESHOLD_TRIM_77_DAC_1 = 448,
        THRESHOLD_TRIM_77_DAC_2 = 449,
        THRESHOLD_TRIM_77_DAC_3 = 450,
        THRESHOLD_TRIM_78_DAC_0 = 451,
        THRESHOLD_TRIM_78_DAC_1 = 452,
        THRESHOLD_TRIM_78_DAC_2 = 453,
        THRESHOLD_TRIM_78_DAC_3 = 454,
        THRESHOLD_TRIM_79_DAC_0 = 455,
        THRESHOLD_TRIM_79_DAC_1 = 456,
        THRESHOLD_TRIM_79_DAC_2 = 457,
        THRESHOLD_TRIM_79_DAC_3 = 458,
        THRESHOLD_TRIM_80_DAC_0 = 459,
        THRESHOLD_TRIM_80_DAC_1 = 460,
        THRESHOLD_TRIM_80_DAC_2 = 461,
        THRESHOLD_TRIM_80_DAC_3 = 462,
        THRESHOLD_TRIM_81_DAC_0 = 463,
        THRESHOLD_TRIM_81_DAC_1 = 464,
        THRESHOLD_TRIM_81_DAC_2 = 465,
        THRESHOLD_TRIM_81_DAC_3 = 466,
        THRESHOLD_TRIM_82_DAC_0 = 467,
        THRESHOLD_TRIM_82_DAC_1 = 468,
        THRESHOLD_TRIM_82_DAC_2 = 469,
        THRESHOLD_TRIM_82_DAC_3 = 470,
        THRESHOLD_TRIM_83_DAC_0 = 471,
        THRESHOLD_TRIM_83_DAC_1 = 472,
        THRESHOLD_TRIM_83_DAC_2 = 473,
        THRESHOLD_TRIM_83_DAC_3 = 474,
        THRESHOLD_TRIM_84_DAC_0 = 475,
        THRESHOLD_TRIM_84_DAC_1 = 476,
        THRESHOLD_TRIM_84_DAC_2 = 477,
        THRESHOLD_TRIM_84_DAC_3 = 478,
        THRESHOLD_TRIM_85_DAC_0 = 479,
        THRESHOLD_TRIM_85_DAC_1 = 480,
        THRESHOLD_TRIM_85_DAC_2 = 481,
        THRESHOLD_TRIM_85_DAC_3 = 482,
        THRESHOLD_TRIM_86_DAC_0 = 483,
        THRESHOLD_TRIM_86_DAC_1 = 484,
        THRESHOLD_TRIM_86_DAC_2 = 485,
        THRESHOLD_TRIM_86_DAC_3 = 486,
        THRESHOLD_TRIM_87_DAC_0 = 487,
        THRESHOLD_TRIM_87_DAC_1 = 488,
        THRESHOLD_TRIM_87_DAC_2 = 489,
        THRESHOLD_TRIM_87_DAC_3 = 490,
        THRESHOLD_TRIM_88_DAC_0 = 491,
        THRESHOLD_TRIM_88_DAC_1 = 492,
        THRESHOLD_TRIM_88_DAC_2 = 493,
        THRESHOLD_TRIM_88_DAC_3 = 494,
        THRESHOLD_TRIM_89_DAC_0 = 495,
        THRESHOLD_TRIM_89_DAC_1 = 496,
        THRESHOLD_TRIM_89_DAC_2 = 497,
        THRESHOLD_TRIM_89_DAC_3 = 498,
        THRESHOLD_TRIM_90_DAC_0 = 499,
        THRESHOLD_TRIM_90_DAC_1 = 500,
        THRESHOLD_TRIM_90_DAC_2 = 501,
        THRESHOLD_TRIM_90_DAC_3 = 502,
        THRESHOLD_TRIM_91_DAC_0 = 503,
        THRESHOLD_TRIM_91_DAC_1 = 504,
        THRESHOLD_TRIM_91_DAC_2 = 505,
        THRESHOLD_TRIM_91_DAC_3 = 506,
        THRESHOLD_TRIM_92_DAC_0 = 507,
        THRESHOLD_TRIM_92_DAC_1 = 508,
        THRESHOLD_TRIM_92_DAC_2 = 509,
        THRESHOLD_TRIM_92_DAC_3 = 510,
        THRESHOLD_TRIM_93_DAC_0 = 511,
        THRESHOLD_TRIM_93_DAC_1 = 512,
        THRESHOLD_TRIM_93_DAC_2 = 513,
        THRESHOLD_TRIM_93_DAC_3 = 514,
        THRESHOLD_TRIM_94_DAC_0 = 515,
        THRESHOLD_TRIM_94_DAC_1 = 516,
        THRESHOLD_TRIM_94_DAC_2 = 517,
        THRESHOLD_TRIM_94_DAC_3 = 518,
        THRESHOLD_TRIM_95_DAC_0 = 519,
        THRESHOLD_TRIM_95_DAC_1 = 520,
        THRESHOLD_TRIM_95_DAC_2 = 521,
        THRESHOLD_TRIM_95_DAC_3 = 522,
        THRESHOLD_TRIM_96_DAC_0 = 523,
        THRESHOLD_TRIM_96_DAC_1 = 524,
        THRESHOLD_TRIM_96_DAC_2 = 525,
        THRESHOLD_TRIM_96_DAC_3 = 526,
        THRESHOLD_TRIM_97_DAC_0 = 527,
        THRESHOLD_TRIM_97_DAC_1 = 528,
        THRESHOLD_TRIM_97_DAC_2 = 529,
        THRESHOLD_TRIM_97_DAC_3 = 530,
        THRESHOLD_TRIM_98_DAC_0 = 531,
        THRESHOLD_TRIM_98_DAC_1 = 532,
        THRESHOLD_TRIM_98_DAC_2 = 533,
        THRESHOLD_TRIM_98_DAC_3 = 534,
        THRESHOLD_TRIM_99_DAC_0 = 535,
        THRESHOLD_TRIM_99_DAC_1 = 536,
        THRESHOLD_TRIM_99_DAC_2 = 537,
        THRESHOLD_TRIM_99_DAC_3 = 538,
        THRESHOLD_TRIM_100_DAC_0 = 539,
        THRESHOLD_TRIM_100_DAC_1 = 540,
        THRESHOLD_TRIM_100_DAC_2 = 541,
        THRESHOLD_TRIM_100_DAC_3 = 542,
        THRESHOLD_TRIM_101_DAC_0 = 543,
        THRESHOLD_TRIM_101_DAC_1 = 544,
        THRESHOLD_TRIM_101_DAC_2 = 545,
        THRESHOLD_TRIM_101_DAC_3 = 546,
        THRESHOLD_TRIM_102_DAC_0 = 547,
        THRESHOLD_TRIM_102_DAC_1 = 548,
        THRESHOLD_TRIM_102_DAC_2 = 549,
        THRESHOLD_TRIM_102_DAC_3 = 550,
        THRESHOLD_TRIM_103_DAC_0 = 551,
        THRESHOLD_TRIM_103_DAC_1 = 552,
        THRESHOLD_TRIM_103_DAC_2 = 553,
        THRESHOLD_TRIM_103_DAC_3 = 554,
        THRESHOLD_TRIM_104_DAC_0 = 555,
        THRESHOLD_TRIM_104_DAC_1 = 556,
        THRESHOLD_TRIM_104_DAC_2 = 557,
        THRESHOLD_TRIM_104_DAC_3 = 558,
        THRESHOLD_TRIM_105_DAC_0 = 559,
        THRESHOLD_TRIM_105_DAC_1 = 560,
        THRESHOLD_TRIM_105_DAC_2 = 561,
        THRESHOLD_TRIM_105_DAC_3 = 562,
        THRESHOLD_TRIM_106_DAC_0 = 563,
        THRESHOLD_TRIM_106_DAC_1 = 564,
        THRESHOLD_TRIM_106_DAC_2 = 565,
        THRESHOLD_TRIM_106_DAC_3 = 566,
        THRESHOLD_TRIM_107_DAC_0 = 567,
        THRESHOLD_TRIM_107_DAC_1 = 568,
        THRESHOLD_TRIM_107_DAC_2 = 569,
        THRESHOLD_TRIM_107_DAC_3 = 570,
        THRESHOLD_TRIM_108_DAC_0 = 571,
        THRESHOLD_TRIM_108_DAC_1 = 572,
        THRESHOLD_TRIM_108_DAC_2 = 573,
        THRESHOLD_TRIM_108_DAC_3 = 574,
        THRESHOLD_TRIM_109_DAC_0 = 575,
        THRESHOLD_TRIM_109_DAC_1 = 576,
        THRESHOLD_TRIM_109_DAC_2 = 577,
        THRESHOLD_TRIM_109_DAC_3 = 578,
        THRESHOLD_TRIM_110_DAC_0 = 579,
        THRESHOLD_TRIM_110_DAC_1 = 580,
        THRESHOLD_TRIM_110_DAC_2 = 581,
        THRESHOLD_TRIM_110_DAC_3 = 582,
        THRESHOLD_TRIM_111_DAC_0 = 583,
        THRESHOLD_TRIM_111_DAC_1 = 584,
        THRESHOLD_TRIM_111_DAC_2 = 585,
        THRESHOLD_TRIM_111_DAC_3 = 586,
        THRESHOLD_TRIM_112_DAC_0 = 587,
        THRESHOLD_TRIM_112_DAC_1 = 588,
        THRESHOLD_TRIM_112_DAC_2 = 589,
        THRESHOLD_TRIM_112_DAC_3 = 590,
        THRESHOLD_TRIM_113_DAC_0 = 591,
        THRESHOLD_TRIM_113_DAC_1 = 592,
        THRESHOLD_TRIM_113_DAC_2 = 593,
        THRESHOLD_TRIM_113_DAC_3 = 594,
        THRESHOLD_TRIM_114_DAC_0 = 595,
        THRESHOLD_TRIM_114_DAC_1 = 596,
        THRESHOLD_TRIM_114_DAC_2 = 597,
        THRESHOLD_TRIM_114_DAC_3 = 598,
        THRESHOLD_TRIM_115_DAC_0 = 599,
        THRESHOLD_TRIM_115_DAC_1 = 600,
        THRESHOLD_TRIM_115_DAC_2 = 601,
        THRESHOLD_TRIM_115_DAC_3 = 602,
        THRESHOLD_TRIM_116_DAC_0 = 603,
        THRESHOLD_TRIM_116_DAC_1 = 604,
        THRESHOLD_TRIM_116_DAC_2 = 605,
        THRESHOLD_TRIM_116_DAC_3 = 606,
        THRESHOLD_TRIM_117_DAC_0 = 607,
        THRESHOLD_TRIM_117_DAC_1 = 608,
        THRESHOLD_TRIM_117_DAC_2 = 609,
        THRESHOLD_TRIM_117_DAC_3 = 610,
        THRESHOLD_TRIM_118_DAC_0 = 611,
        THRESHOLD_TRIM_118_DAC_1 = 612,
        THRESHOLD_TRIM_118_DAC_2 = 613,
        THRESHOLD_TRIM_118_DAC_3 = 614,
        THRESHOLD_TRIM_119_DAC_0 = 615,
        THRESHOLD_TRIM_119_DAC_1 = 616,
        THRESHOLD_TRIM_119_DAC_2 = 617,
        THRESHOLD_TRIM_119_DAC_3 = 618,
        THRESHOLD_TRIM_120_DAC_0 = 619,
        THRESHOLD_TRIM_120_DAC_1 = 620,
        THRESHOLD_TRIM_120_DAC_2 = 621,
        THRESHOLD_TRIM_120_DAC_3 = 622,
        THRESHOLD_TRIM_121_DAC_0 = 623,
        THRESHOLD_TRIM_121_DAC_1 = 624,
        THRESHOLD_TRIM_121_DAC_2 = 625,
        THRESHOLD_TRIM_121_DAC_3 = 626,
        THRESHOLD_TRIM_122_DAC_0 = 627,
        THRESHOLD_TRIM_122_DAC_1 = 628,
        THRESHOLD_TRIM_122_DAC_2 = 629,
        THRESHOLD_TRIM_122_DAC_3 = 630,
        THRESHOLD_TRIM_123_DAC_0 = 631,
        THRESHOLD_TRIM_123_DAC_1 = 632,
        THRESHOLD_TRIM_123_DAC_2 = 633,
        THRESHOLD_TRIM_123_DAC_3 = 634,
        THRESHOLD_TRIM_124_DAC_0 = 635,
        THRESHOLD_TRIM_124_DAC_1 = 636,
        THRESHOLD_TRIM_124_DAC_2 = 637,
        THRESHOLD_TRIM_124_DAC_3 = 638,
        THRESHOLD_TRIM_125_DAC_0 = 639,
        THRESHOLD_TRIM_125_DAC_1 = 640,
        THRESHOLD_TRIM_125_DAC_2 = 641,
        THRESHOLD_TRIM_125_DAC_3 = 642,
        THRESHOLD_TRIM_126_DAC_0 = 643,
        THRESHOLD_TRIM_126_DAC_1 = 644,
        THRESHOLD_TRIM_126_DAC_2 = 645,
        THRESHOLD_TRIM_126_DAC_3 = 646,
        THRESHOLD_TRIM_127_DAC_0 = 647,
        THRESHOLD_TRIM_127_DAC_1 = 648,
        THRESHOLD_TRIM_127_DAC_2 = 649,
        THRESHOLD_TRIM_127_DAC_3 = 650
    }

    #endregion

    public record SRE3021ImageData(int CathodeValue, int CathodeTiming, int[,] AnodeValue, int[,] AnodeTiming);
    public static class SRE3021API
    {
        public static List<SRE3021SysReg> SRE3021SysRegisters = new List<SRE3021SysReg>();

        static SRE3021API()
        {
            IMGDataEventRecieved += ProcessImgData;
            IsPorcessImagDataSubscribed = true;
        }

        /// <summary>
        /// Initiate SRE3021API
        /// </summary>
        /// <returns>If success to Initiate return true</returns>
        static public bool InitiateSRE3021API()
        {
            if (!IsTCPOpen)
            {
                return false;
            }
            OpenUDPPort();
            ReadAllSysRegs();

            InitASICConfigBits();
            CheckBaseline();
            try
            {
                Console.WriteLine("Initiate CZT Done");
            }
            catch
            {

            }
            mIsInitiate = true;
            return true;
        }



        #region Network Variables , Fuctions
        private static readonly IPAddress HostIP = IPAddress.Parse("10.10.0.100");
        private static readonly IPAddress SRE3021IP = IPAddress.Parse("10.10.0.50");
        private const int SRE3021TCPPort = 50010;
        private const int SRE3021UDPPort = 50011;
        private const int MTU = 15000;
        public static int UDPPacketCount = 0;

        private static TcpClient TCPSocket;

        private static UdpClient UDPSocket;

        private static NetworkStream TCPNetworkStream;
        private static byte[] TCPNetworkStreamBuffer = new byte[15000];
        public static bool IsTCPOpen
        {
            get
            {
                /// Ping Check
                Ping ping = new Ping();



                PingOptions options = new PingOptions();

                options.DontFragment = true;

                //전송할 데이터를 입력

                string data = "aaaaaaaaaaaaaa";

                byte[] buffer = ASCIIEncoding.ASCII.GetBytes(data);

                int timeout = 120;



                //IP 주소를 입력

                PingReply reply = ping.Send(SRE3021IP, timeout, buffer, options);



                if (reply.Status == IPStatus.Success)
                {
                    try
                    {
                        Console.WriteLine("CZT Ping Succeess");
                    }
                    catch
                    {

                    }
                }
                else

                {
                    Console.WriteLine("CZT Ping Fail");
                    return false;
                }

                TCPSocket = new TcpClient();
                TCPSocket.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
                LingerOption lo = new LingerOption(true, 0);
                TCPSocket.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Linger, lo);

                try
                {
                    TCPSocket.Connect(new IPEndPoint(SRE3021IP, SRE3021TCPPort));
                }
                catch
                {
                    return false;
                }
                CloseTCPPort();
                return true;
            }
        }

        public static bool IsUDPOpen
        {
            get
            {
                if (UDPSocket == null)
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
        }

        /// <summary>
        /// Try open TCP Port for SRE3021
        /// </summary>
        /// <returns>true if success</returns>
        public static bool OpenTCPPort()
        {
            TCPSocket = new TcpClient();
            TCPSocket.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            LingerOption lo = new LingerOption(true, 0);
            TCPSocket.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Linger, lo);
            TCPSocket.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, 500);


            TCPSocket.Connect(new IPEndPoint(SRE3021IP, SRE3021TCPPort));

            TCPNetworkStream = TCPSocket.GetStream();


            if (!TCPSocket.Connected)
            {
                Trace.WriteLine("TCP Connection Fail");
                TCPSocket.Close();
                TCPSocket = null;
                return false;
            }
            return true;
        }

        public static void CloseTCPPort()
        {
            if (TCPSocket == null)
            {
                return;
            }
            if (TCPNetworkStream != null)
            {
                TCPNetworkStream.Close();
            }
            TCPSocket.Close();
            TCPSocket = null;
        }

        /// <summary>
        /// Try open UDP port for SRE3021
        /// </summary>
        /// <returns>false if fail</returns>
        public static bool OpenUDPPort()
        {
            if (IsUDPOpen)
            {
                return true;
            }
            UDPSocket = new UdpClient(SRE3021UDPPort);

            UDPListener();

            return true;
        }
        private static void UDPListener()
        {
            var epUdp = new IPEndPoint(SRE3021IP, SRE3021UDPPort);
            Task.Run(() =>
            {
                while (UDPSocket != null)
                {
                    //IPEndPoint object will allow us to read datagrams sent from any source.
                    try
                    {
                        var receivedResults = UDPSocket.Receive(ref epUdp);
                        UDPImagBuffer.Add(receivedResults);
                        Thread.Sleep(0);
                    }
                    catch
                    {
                        break;
                    }

                }
            });
            Task.Run(() =>
            {
                while (UDPSocket != null)
                {
                    byte[] buffer;
                    while (UDPImagBuffer.TryTake(out buffer))
                    {
                        RaiseReadImagDataEvent(buffer);
                    }
                    Thread.Sleep(0);
                }
            });

        }

        private static BlockingCollection<byte[]> UDPImagBuffer = new BlockingCollection<byte[]>();

        #endregion

        #region Setting System Regs
        private static void ReadAllSysRegs()
        {
            SRE3021SysRegisters = new List<SRE3021SysReg>();
            foreach (SRE3021SysRegisterADDR addr in Enum.GetValues(typeof(SRE3021SysRegisterADDR)))
            {
                SRE3021SysRegisters.Add(ReadSysReg(addr));
            }

            ReadWriteASICReg(SRE3021ASICRegisterADDR.NOTSELECT, false);
        }
        public static SRE3021SysReg ReadSysReg(SRE3021SysRegisterADDR address)
        {
            OpenTCPPort();

            SRE3021PacketHeader pHeader = new SRE3021PacketHeader(SRE3021PacketType.READ_SYS_REG, 2);
            byte[] data = BitConverter.GetBytes((UInt16)address);
            var sendData = new List<byte>();
            sendData.AddRange(pHeader.ByteData);
            Array.Reverse(data);
            sendData.AddRange(data);
            TCPNetworkStream.Write(sendData.ToArray(), 0, sendData.Count);
            TCPNetworkStream.Flush();
            byte[] checkBytes = new byte[MTU];
            TCPNetworkStream.Read(checkBytes, 0, MTU);
            CloseTCPPort();
            return DecodeReadbackSysReg(checkBytes);
        }
        public static SRE3021SysReg WriteSysReg(SRE3021SysRegisterADDR address, int value)
        {
            OpenTCPPort();

            if (SRE3021SysReg.CheckType(address) == RWType.R)
            {
                return null;
            }

            byte[] regAddress = BitConverter.GetBytes((UInt16)address);
            Array.Reverse(regAddress);

            int registerLength = SRE3021SysReg.GetByteLength(address);

            byte[] regData;
            switch (registerLength)
            {
                case 1:
                    regData = new byte[] { (byte)value };
                    break;
                case 2:
                    regData = BitConverter.GetBytes((UInt16)value);
                    break;
                case 4:
                    regData = BitConverter.GetBytes((UInt32)value);
                    break;
                case 8:
                    regData = BitConverter.GetBytes((UInt64)value);
                    break;
                default:
                    throw new ArgumentException();
            }
            Array.Reverse(regData);

            var sendData = new List<byte>();

            SRE3021PacketHeader pHeader = new SRE3021PacketHeader(SRE3021PacketType.WRITE_SYS_REG, (ushort)(registerLength + 3));

            sendData.AddRange(pHeader.ByteData);
            sendData.AddRange(regAddress);
            sendData.Add((byte)registerLength);
            sendData.AddRange(regData);


            byte[] checkBytes = new byte[MTU];

            TCPNetworkStream.Write(sendData.ToArray(), 0, sendData.Count);
            TCPNetworkStream.Flush();
            TCPNetworkStream.Read(checkBytes, 0, MTU);
            CloseTCPPort();
            return DecodeReadbackSysReg(checkBytes);
        }
        private static SRE3021SysReg DecodeReadbackSysReg(byte[] bytes)
        {
            SRE3021PacketHeader pHeader = new SRE3021PacketHeader(bytes[0..10]);
            Debug.Assert(pHeader.PacketType == SRE3021PacketType.READBACK_SYS_REG);
            byte[] data = new byte[pHeader.DataLength];
            Array.Copy(bytes, 10, data, 0, pHeader.DataLength);
            SRE3021SysRegisterADDR addr = (SRE3021SysRegisterADDR)BitConverter.ToUInt16(new byte[] { data[1], data[0] });
            int regLegth = (int)data[2];
            long regData;
            switch (regLegth)
            {
                case 1:
                    regData = (int)data[3];
                    break;
                case 2:
                    regData = BitConverter.ToInt16(new byte[] { data[4], data[3] });
                    break;
                case 3:
                    regData = BitConverter.ToInt32(new byte[] { data[5], data[4], data[3], 0x00 });
                    break;
                case 4:
                    regData = BitConverter.ToInt32(new byte[] { data[6], data[5], data[4], data[3] });
                    break;
                case 8:
                    regData = BitConverter.ToInt64(new byte[] { data[10], data[9], data[8], data[7], data[6], data[5], data[4], data[3] });
                    break;
                default:
                    throw new ArgumentException();
            }
            var sysReg = new SRE3021SysReg(addr, (uint)(regLegth * 8));
            sysReg.Value = (long)regData;
            return sysReg;
        }

        #endregion

        #region Setting ASIC regs
        private const int ASICBitSize = 650;
        private static bool[] ASICConfigBits = new bool[ASICBitSize];

        private static void InitASICConfigBits()
        {
            byte[] initBytes = new byte[] {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                          0x00, 0x00, 0x1b, 0x20, 0x00 };
            DecodeReadbackASICReg(initBytes);




            for (int j = 1; j <= 650; ++j)
            {
                bool result = GetASICConfigtBitValue((SRE3021ASICRegisterADDR)j);
                Debug.WriteLine("{0} is {1}", ((SRE3021ASICRegisterADDR)j).ToString(), result);
            }
        }
        public static void ReadWriteASICReg(SRE3021ASICRegisterADDR addr, bool value)
        {
            OpenTCPPort();

            SRE3021PacketHeader pHeader = new SRE3021PacketHeader(SRE3021PacketType.RW_ASIC_REG, 85);
            byte[] data = new byte[82];
            if (addr != SRE3021ASICRegisterADDR.NOTSELECT)
            {
                ASICConfigBits[(650 - (int)addr)] = value;
            }

            bool[] changedConfig = new bool[650];
            ASICConfigBits.CopyTo(changedConfig, 0);
            for (int i = 0; i < 81; ++i)
            {
                data[i] = ConvertBoolArrayToByte(ASICConfigBits[((i * 8) + 0)..((i * 8) + 8)]);
            }
            data[81] = ConvertBoolArrayToByte(ASICConfigBits[((81 * 8) + 0)..650]);



            var sendData = new List<byte>();
            sendData.AddRange(pHeader.ByteData);
            sendData.AddRange(new byte[] { 0x00, 0x02, 0x8a });
            sendData.AddRange(data);

            byte[] checkBytes = new byte[MTU];

            //Error fix -> readback twice to set ASIC properly
            TCPNetworkStream.Write(sendData.ToArray(), 0, sendData.Count);
            TCPNetworkStream.Flush();
            TCPNetworkStream.Read(checkBytes, 0, MTU);
            TCPNetworkStream.Write(sendData.ToArray(), 0, sendData.Count);
            TCPNetworkStream.Flush();
            TCPNetworkStream.Read(checkBytes, 0, MTU);

            DecodeReadbackASICReg(checkBytes[13..]);

            for (int i = 0; i < 650; ++i)
            {
                if (changedConfig[i] != ASICConfigBits[i])
                {
                    throw new Exception("ASIC Config Failed");
                }
            }

            CloseTCPPort();
        }
        private static void DecodeReadbackASICReg(byte[] bytes)
        {
            int i = 0;
            foreach (var b in bytes)
            {

                BitArray bits = new BitArray(new byte[] { b });
                for (int j = 0; j < 8; ++j)
                {
                    if (i == 650)
                    {
                        break;
                    }
                    ASICConfigBits[i] = bits[7 - j];
                    ++i;

                }
            }
        }
        private static byte ConvertBoolArrayToByte(bool[] source)
        {
            byte result = 0;
            // This assumes the array never contains more than 8 elements!
            int index = 0;

            // Loop through the array
            foreach (bool b in source)
            {
                // if the element is 'true' set the bit at that position
                if (b)
                {
                    result |= (byte)(1 << (7 - index));
                }

                index++;
            }

            return result;
        }
        public static bool GetASICConfigtBitValue(SRE3021ASICRegisterADDR addr)
        {
            return ASICConfigBits[650 - (int)addr];
        }
        #endregion

        #region Handle IMGDataStream
        public delegate void ReadIMGDataEventHanlder(SRE3021ImageData imageData);
        public static event ReadIMGDataEventHanlder IMGDataEventRecieved;
        private static void RaiseReadImagDataEvent(byte[] bytes)
        {
            if (IMGDataEventRecieved == null)
            {
                return;
            }
            var pHeader = new SRE3021PacketHeader(bytes[0..10]);
            if (pHeader.PacketType != SRE3021PacketType.IMG_DATA)
            {
                return;
            }

            int catE = (int)BitConverter.ToUInt16(new byte[] { bytes[21], bytes[20] }) - CathodeValueBaseline;
            int catT = (int)BitConverter.ToUInt16(new byte[] { bytes[23], bytes[22] }) - CathodeTimingBaseline;
            var anodeE = new int[11, 11];
            var anodeT = new int[11, 11];
            int UnusableChannel1 = ASICChannelNumber[2, 0];
            int UnusableChannel2 = ASICChannelNumber[0, 0];
            int imgOrder = 0;
            for (int Y = 0; Y < 11; ++Y)
            {
                for (int X = 0; X < 11; ++X)
                {
                    if (Y == 0 && (X == 2 || X == 0))
                    {
                        imgOrder += 2;
                        continue;
                    }
                    anodeE[X, Y] = (int)BitConverter.ToUInt16(new byte[] { bytes[31 + imgOrder], bytes[30 + imgOrder] }) - AnodeValueBaseline[X, Y];
                    anodeT[X, Y] = (int)BitConverter.ToUInt16(new byte[] { bytes[273 + imgOrder], bytes[272 + imgOrder] }) - AnodeTimingBaseline[X, Y];
                    imgOrder += 2;

                }
            }
            ++UDPPacketCount;

            SRE3021ImageData imageData = new SRE3021ImageData(catE, catT, anodeE, anodeT);

            IMGDataEventRecieved.Invoke(imageData);
        }


        private static bool IsPorcessImagDataSubscribed = false;
        public static void UnhandleDefaultProcessImagData()
        {
            if (IsPorcessImagDataSubscribed)
            {
                IMGDataEventRecieved -= ProcessImgData;
            }
        }
        private static void ProcessImgData(SRE3021ImageData imgData)
        {
            const double p1 = 0.2987;
            const double p2 = 49.72;

            List<int> interactionX = new List<int>();
            List<int> interactionY = new List<int>();
            int interactionPotins = 0;
            int backgroundNoise = 0;

            for (int X = 0; X < 11; ++X)
            {
                for (int Y = 0; Y < 11; ++Y)
                {
                    if (imgData.AnodeTiming[X, Y] > 150)
                    {
                        ++interactionPotins;
                        if (interactionPotins == 3)
                        {
                            return;
                        }
                        interactionX.Add(X);
                        interactionY.Add(Y);
                    }
                    else
                    {
                        backgroundNoise += imgData.AnodeValue[X, Y];
                    }

                }
            }
            if (interactionX.Count == 0)
            {
                return;
            }
            else if (interactionX.Count == 1)
            {
                backgroundNoise = backgroundNoise / 120;

                SpectrumEnergy.AddEnergy((Convert.ToDouble(imgData.AnodeValue[interactionX[0], interactionY[0]]) - backgroundNoise) * p1 + p2);
                SpectrumEnergyIsoFind.AddEnergy((Convert.ToDouble(imgData.AnodeValue[interactionX[0], interactionY[0]]) - backgroundNoise) * p1 + p2);
            }
        }

        #endregion

        #region BaseLine Check

        public static readonly int[,] ChannelNumber =
        {
            {3, 7, 12, 4, 20, 10, 115, 109, 114, 106, 111},
            {11, 14, 2, 18, 6, 101, 105, 98, 116, 118, 112},
            {1, 24, 16, 0, 8, 113, 119, 120, 107, 108, 110},
            {5, 9, 15, 22, 103, 117, 100, 76, 84, 102, 104},
            {17, 13, 23, 44, 56, 64, 68, 72, 80, 97, 96},
            {19, 21, 40, 48, 52, 60, 66, 74, 82, 99, 95},
            {25, 29, 38, 46, 54, 58, 62, 70, 78, 86, 94},
            {27, 26, 36, 42, 50, 53, 61, 77, 89, 91, 90},
            {31, 32, 35, 41, 47, 55, 67, 75, 93, 87, 88},
            {30, 34, 37, 43, 49, 59, 65, 69, 79, 81, 92},
            {28, 33, 39, 45, 51, 57, 63, 71, 73, 83, 85}
        };

        public static readonly int[,] ASICChannelNumber =
        {
            {8, 12, 17, 9, 25, 15, 122, 116, 121, 113, 118},
            {16, 19, 7, 23, 11, 108, 112, 105, 123, 125, 119},
            {6, 29, 21, 5, 13, 120, 126, 127, 114, 115, 117},
            {10, 14, 20, 27, 110, 124, 107, 83, 91, 109, 111},
            {22, 18, 28, 51, 63, 71, 75, 79, 87, 104, 103},
            {24, 26, 47, 55, 59, 67, 73, 81, 89, 106, 102},
            {30, 36, 45, 53, 61, 65, 69, 77, 85, 93, 101},
            {33, 32, 43, 49, 57, 60, 68, 84, 96, 98, 97},
            {38, 39, 42, 48, 54, 62, 74, 82, 100, 94, 95},
            {37, 41, 44, 50, 56, 66, 72, 76, 86, 88, 99},
            {35, 40, 46, 52, 58, 64, 70, 78, 80, 90, 92}
        };
        public static int CathodeValueBaseline = 0;
        public static int CathodeTimingBaseline = 0;
        public static int[,] AnodeValueBaseline = new int[11, 11];
        public static int[,] AnodeTimingBaseline = new int[11, 11];

        public static bool IsBaselineSet
        {
            get
            {
                if (AnodeValueBaseline[0, 0] != 0)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        public static void CheckBaseline()
        {
            int numPulses = 10000;

            Trace.WriteLine("Configuring ASIC.");
            ReadWriteASICReg(SRE3021ASICRegisterADDR.Test_on, false);
            ReadWriteASICReg(SRE3021ASICRegisterADDR.Current_compensation_enable, true);

            Trace.WriteLine("Configuring system.");
            WriteSysReg(SRE3021SysRegisterADDR.CFG_FIXED_CH, 128); //This vale ensures that single channel readout is not envoked during baseline readout.
            WriteSysReg(SRE3021SysRegisterADDR.CFG_HOLD_DLY, 1); //This is the minimum value for the peak-hold to pick up less noise.
            WriteSysReg(SRE3021SysRegisterADDR.CFG_CALTRIG_EN, 1);

            Trace.WriteLine("Configuring calibration pulse genenerator (CalGen).");
            WriteSysReg(SRE3021SysRegisterADDR.CAL_PULSE_POLAR, 0);
            WriteSysReg(SRE3021SysRegisterADDR.CAL_NUM_PULSE, numPulses);
            WriteSysReg(SRE3021SysRegisterADDR.CAL_PULSE_LENGTH, 500);
            WriteSysReg(SRE3021SysRegisterADDR.CAL_PULSE_INTERVAL, 40000);

            Trace.WriteLine("Start pulse genenerator (CalGen).");

            Stopwatch sw = new Stopwatch();
            IMGDataEventRecieved += BaseLineEventCheck;
            WriteSysReg(SRE3021SysRegisterADDR.CAL_EXECUTE, 1);

            sw.Start();

            while (BaseLineImageEvents.Count < numPulses)
            {

                if (sw.ElapsedMilliseconds > 15000)
                {
                    Trace.WriteLine($"BaseLineImageEvents Count is {BaseLineImageEvents.Count}");
                    if (BaseLineImageEvents.Count > numPulses * 0.99)
                    {
                        break;
                    }
                    throw new TimeoutException();
                }
            }
            sw.Stop();
            Trace.WriteLine($"Baseline Cal Execute time is {sw.ElapsedMilliseconds} [ms]");
            IMGDataEventRecieved -= BaseLineEventCheck;

            int[,] baselineSum = new int[11, 11];
            int[,] baselineTimingSum = new int[11, 11];
            int catbaslineSum = 0;
            int catTimingbaselineSum = 0;
            foreach (var imageData in BaseLineImageEvents)
            {
                catbaslineSum += imageData.CathodeValue;
                catTimingbaselineSum += imageData.CathodeTiming;
                for (int i = 0; i < 11; ++i)
                {
                    for (int j = 0; j < 11; ++j)
                    {
                        baselineSum[i, j] += imageData.AnodeValue[i, j];
                        baselineTimingSum[i, j] += imageData.AnodeTiming[i, j];
                    }
                }
            }
            CathodeValueBaseline = catbaslineSum / BaseLineImageEvents.Count;
            CathodeTimingBaseline = catTimingbaselineSum / BaseLineImageEvents.Count;
            for (int i = 0; i < 11; ++i)
            {
                for (int j = 0; j < 11; ++j)
                {
                    AnodeValueBaseline[i, j] = baselineSum[i, j] / BaseLineImageEvents.Count;
                    AnodeTimingBaseline[i, j] = baselineTimingSum[i, j] / BaseLineImageEvents.Count;
                }
            }

            BaseLineImageEvents.Clear();


        }
        private static List<SRE3021ImageData> BaseLineImageEvents = new List<SRE3021ImageData>();
        private static void BaseLineEventCheck(SRE3021ImageData imageData)
        {
            BaseLineImageEvents.Add(imageData);
        }

        #endregion

        #region Acquire Data
        static private SpectrumEnergyNasa SpectrumEnergy = new SpectrumEnergyNasa(5, 2000);
        static private SpectrumEnergyNasa SpectrumEnergyIsoFind = new SpectrumEnergyNasa(5, 2000);



        private static bool mIsInitiate = false;
        public static bool IsAPIAvailable => mIsInitiate && IsTCPOpen;

        public static void ResetSpectrumEnergy()
        {
            SpectrumEnergy = new SpectrumEnergyNasa(5, 2000);
            SpectrumEnergyIsoFind = new SpectrumEnergyNasa(5, 2000);
        }

        public static SpectrumEnergyNasa GetSpectrumEnergy
        {
            get
            {
                return SpectrumEnergy;
            }
        }

        public static SpectrumEnergyNasa GetSpectrumEnergyIsoFind
        {
            get
            {
                return SpectrumEnergyIsoFind;
            }
        }


        public static int SetHighVoltage(int voltage, int step, int sleepTimeInMillisecond)
        {
            int CurrentVoltage = (int)ReadSysReg(SRE3021SysRegisterADDR.HV_DAC).Value;

            if (CurrentVoltage > voltage)
            {
                int setVoltage = CurrentVoltage;
                while (voltage < (int)ReadSysReg(SRE3021SysRegisterADDR.HV_DAC).Value)
                {
                    if (voltage + step < CurrentVoltage)
                    {
                        setVoltage -= step;
                    }
                    else
                    {
                        setVoltage = voltage;
                    }

                    CurrentVoltage = (int)WriteSysReg(SRE3021SysRegisterADDR.HV_DAC, setVoltage).Value;
                    Debug.WriteLine($"Voltage drop to {CurrentVoltage}");
                    Thread.Sleep(sleepTimeInMillisecond);
                }
            }
            else
            {
                int setVoltage = CurrentVoltage;
                while (voltage > (int)ReadSysReg(SRE3021SysRegisterADDR.HV_DAC).Value)
                {
                    if (voltage - step > CurrentVoltage)
                    {
                        setVoltage += step;
                    }
                    else
                    {
                        setVoltage = voltage;
                    }

                    CurrentVoltage = (int)WriteSysReg(SRE3021SysRegisterADDR.HV_DAC, setVoltage).Value;
                    Debug.WriteLine($"Voltage rise to {CurrentVoltage}");
                    Thread.Sleep(sleepTimeInMillisecond);
                }
            }
            return CurrentVoltage;
        }
        public static void StartAcqusition(int HV = 1500, int VTHR = 2435, int VTHR0 = 2457, int Hold_DLY = 300, int VFP0 = 1750)
        {

            ReadWriteASICReg(SRE3021ASICRegisterADDR.Test_on, false);
            ReadWriteASICReg(SRE3021ASICRegisterADDR.Test_on, false);
            ReadWriteASICReg(SRE3021ASICRegisterADDR.Current_compensation_enable, true);
            WriteSysReg(SRE3021SysRegisterADDR.VTHR, VTHR);
            WriteSysReg(SRE3021SysRegisterADDR.VTHR0, VTHR0);
            WriteSysReg(SRE3021SysRegisterADDR.VFP0, VFP0);
            WriteSysReg(SRE3021SysRegisterADDR.CFG_PHYSTRIG_EN, 0);
            WriteSysReg(SRE3021SysRegisterADDR.CMD_SET_CH, 0);
            WriteSysReg(SRE3021SysRegisterADDR.CFG_CALTRIG_EN, 0);
            WriteSysReg(SRE3021SysRegisterADDR.CFG_HOLD_DLY, Hold_DLY); //default 300
            SetHighVoltage(HV, 10, 100);


            WriteSysReg(SRE3021SysRegisterADDR.CFG_PHYSTRIG_EN, 1);

        }
        public static void StopAcqusition()
        {
            WriteSysReg(SRE3021SysRegisterADDR.CFG_PHYSTRIG_EN, 0);
            SetHighVoltage(0, 10, 100);
        }

        #endregion

        public static void Close()
        {
            Trace.WriteLine("SRE3021API Close");
            if (TCPSocket != null)
            {
                TCPNetworkStream.Close();
                TCPSocket.Close();
            }
            if (UDPSocket != null)
            {
                UDPSocket.Close();
                UDPSocket = null;
            }
        }

    }
}