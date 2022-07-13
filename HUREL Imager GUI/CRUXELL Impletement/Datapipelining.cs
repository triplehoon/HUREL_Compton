using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Threading.Tasks.Dataflow;

namespace HUREL.Compton
{
    public partial class CRUXELLLACC
    {

        public BlockingCollection<ushort[]> ShortArrayQueue = new BlockingCollection<ushort[]>();

        //private void ParsingCyusbBufferOrign()
        //{
        //    Console.WriteLine("HY : ParsingThread start");
        //    BinaryWriter writer = new BinaryWriter(File.Open(FileMainPath, FileMode.Append));
        //    while (IsParsing)
        //    {
        //        byte[] Item;

        //        while (DataInQueue.TryTake(out Item))
        //        { 
        //               writer.Write(Item);

        //            Thread.Sleep(0); 
        //            int test_buffercount = DataInQueue.Count;
        //            if (test_buffercount > 1000 && test_buffercount % 1000 == 0)
        //                Console.WriteLine("test buffer count is " + test_buffercount);
        //        }
        //    }
        //    writer.Close();
        //    writer.Dispose();


        //}
        
        
        public bool IsSavingBinaryData;
        private void ParsingCyusbBuffer()
        {

            short flag = 0; //0 is nothing, 1 is find FE, 2 find second FE
            short countflag = 0;

            byte[] dataBuffer = new byte[296];
            byte[] chk1 = new byte[296];
            byte[] chk2 = new byte[296];

            Int64 dataInCount = 0;
            Debug.WriteLine("HY : ParsingThread start");
            BinaryWriter writer = new BinaryWriter(File.Open(FileMainPath, FileMode.Append));
            while (IsParsing)
            {
              
                byte[] item;
                while (DataInQueue.TryTake(out item))
                {


                    if(flag==2 && IsSavingBinaryData)
                    { 
                        writer.Write(item);
                        if (DataInQueue.Count + 1 % 20==0)
                        {
                            Trace.WriteLine("DataInQueCount is " + DataInQueue.Count);
                        }
                    }

                    foreach (byte b in item)
                    {
                        if (flag == 2)
                        {
                            dataBuffer[countflag] = b;
                            countflag++;
                            if (countflag == 296 && dataBuffer[294] == 0xFE && dataBuffer[295] == 0xFE)
                            {
                                //dataBuffer.CopyTo(chk1, 0);
                                //bool checkSame = true;

                                //for (int i = 0; i < 296; ++i)
                                //{
                                //    if (chk1[i] != chk2[i])
                                //    {
                                //        checkSame = false;
                                //        break;
                                //    }
                                //}
                                
                                ParsedQueue.TryAdd(dataBuffer);
                                dataInCount++;
                                
                                //dataBuffer.CopyTo(chk2,0);

                                if(dataInCount % 100000 ==0)
                                {
                                    Trace.WriteLine("Data in count is " + dataInCount + " DataQueue Count: " + ParsedQueue.Count + " ShortArrayBuffer Count: " +ShortArrayQueue.Count);
                                }
                                dataBuffer = new byte[296];
                                countflag = 0;
                            }
                            else if (countflag == 296)
                            {
                                countflag = 0;
                                //dataBuffer = new byte[296];
                            }
                        }
                        else
                        {
                            if (b == 0xFE && flag == 0)
                            {
                                Debug.WriteLine("flag is 1");
                                flag = 1;
                            }
                            else if (b == 0xFE && flag == 1)
                            {
                                Debug.WriteLine("flag is 2");
                                flag = 2;
                            }
                            else
                            {
                                flag = 0;
                                Debug.WriteLine("flag is 0");
                            }
                        }
                    }             
                }              
            }
            writer.Flush();
            writer.Close();
            writer.Dispose();
        }

        private enum ShortBufferMode
        {
            Coin = 0,
            Single = 1,
            SingleCoin1 = 2,
            SingelCoin2 = 3

        }


        private bool IsGenerateShortArrayBuffer;
        private void GenerateShortArrayBuffer_Single()
        {


            while (IsGenerateShortArrayBuffer)
            {
                byte[] item;
                ushort[] shortCheck = new ushort[1];


                while (ParsedQueue.TryTake(out item))
                {
                    ushort[] shortArray = new ushort[144];
                    Buffer.BlockCopy(item, 288, shortCheck, 0, 2);
                    Buffer.BlockCopy(item, shortCheck[0] * 18, shortArray, shortCheck[0] * 18, 18);
                    bool is511Checked = false;
                    foreach (var check511 in shortArray)
                    {
                        if (check511 == 511)
                        {
                            is511Checked = true;
                            continue;
                        }
                    }
                    if (is511Checked)
                    {
                        continue;
                    }
                    ShortArrayQueue.Add(shortArray);
                }
            }
        }


        private void GenerateShortArrayBuffer_Coin()
        {


            while (IsGenerateShortArrayBuffer)
            {
                byte[] item;
                ushort[] shortArray = new ushort[144];


                while (ParsedQueue.TryTake(out item))
                {

                    Buffer.BlockCopy(item, 0, shortArray, 0, 288);
                    foreach (var check511 in shortArray)
                    {
                        if (check511 == 511)
                        {
                            continue;
                        }
                    }
                    ShortArrayQueue.Add(shortArray);
                }
            }
        }

        private void GenerateShortArrayBuffer_SingleCoin1()
        {
            #region BinaryCheck
            ushort[] binaryCheck = new ushort[16];
            binaryCheck[0]  = 0b0000_0000_0000_0001;
            binaryCheck[1]  = 0b0000_0000_0000_0010;
            binaryCheck[2]  = 0b0000_0000_0000_0100;
            binaryCheck[3]  = 0b0000_0000_0000_1000;
            binaryCheck[4]  = 0b0000_0000_0001_0000;
            binaryCheck[5]  = 0b0000_0000_0010_0000;
            binaryCheck[6]  = 0b0000_0000_0100_0000;
            binaryCheck[7]  = 0b0000_0000_1000_0000;
            binaryCheck[8]  = 0b0000_0001_0000_0000;
            binaryCheck[9]  = 0b0000_0010_0000_0000;
            binaryCheck[10] = 0b0000_0100_0000_0000;
            binaryCheck[11] = 0b0000_1000_0000_0000;
            binaryCheck[12] = 0b0001_0000_0000_0000;
            binaryCheck[13] = 0b0010_0000_0000_0000;
            binaryCheck[14] = 0b0100_0000_0000_0000;
            binaryCheck[15] = 0b1000_0000_0000_0000;
            #endregion
            while (IsGenerateShortArrayBuffer)
            {

                byte[] item;



                while (ParsedQueue.TryTake(out item))
                {
                    ushort[] shortArray = new ushort[148];
                    ushort[] shortArray2 = new ushort[144];


                    
                    Buffer.BlockCopy(item, 0, shortArray, 0, 296);
                    ushort check = shortArray[144];
                    //ushort[] test = new ushort[8] { shortArray[0], shortArray[9], shortArray[18], shortArray[27], shortArray[72], shortArray[81], shortArray[90], shortArray[99] };
                    int i = 0;
                    foreach (var b in binaryCheck)
                    {
                        if ((b & check) == 0) {
                            for (int j = 0; j < 9; ++j)
                            {
                                shortArray[j + 9 * (i)] = 0;
                            }
                        }
                        ++i;
                    }


                    ShortArrayQueue.Add(shortArray[0..144]);
                    
                }
            }
        }
        private void GenerateShortArrayBuffer_SingleCoin2()
        {



            while (IsGenerateShortArrayBuffer)
            {
                byte[] item;
                ushort[] shortArray = new ushort[144];
                ushort[] shortArrayTest = new ushort[148];


                while (ParsedQueue.TryTake(out item))
                {

                    Buffer.BlockCopy(item, 0, shortArrayTest, 0, 296);
                    Buffer.BlockCopy(item, 0, shortArray, 0, 288);
                    foreach (var check511 in shortArray)
                    {
                        if (check511 == 511)
                        {
                            continue;
                        }
                    }
                    ShortArrayQueue.Add(shortArray);
                }
            }
        }




    }


}

   

        

