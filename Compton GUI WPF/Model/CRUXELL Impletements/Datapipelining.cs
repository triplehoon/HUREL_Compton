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

        public BlockingCollection<int[]> ListModeData = new BlockingCollection<int[]>();

        private void ParsingCyusbBufferOrign()
        {
            Console.WriteLine("HY : ParsingThread start");
            BinaryWriter writer = new BinaryWriter(File.Open(FileMainPath, FileMode.Append));
            while (IsParsing)
            {
                byte[] Item;

                while (DataInQueue.TryTake(out Item))
                { 
                       writer.Write(Item);

                    Thread.Sleep(0); 
                    int test_buffercount = DataInQueue.Count;
                    if (test_buffercount > 1000 && test_buffercount % 1000 == 0)
                        Console.WriteLine("test buffer count is " + test_buffercount);
                }
            }
            writer.Close();
            writer.Dispose();


        }
        private void ParsingCyusbBufferAsync()
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

                    chk1 = item;
                    if (chk1 == chk2)
                    {
                        Debug.WriteLine(DataInQueue.Count);
                    } 
                    chk2 = item;
                    if(flag==2)
                    { 
                        writer.Write(item);
                        Debug.WriteLine(DataInQueue.Count);
                    }

                    foreach (byte b in item)
                    {
                        if (flag == 2)
                        {
                            dataBuffer[countflag] = b;
                            countflag++;
                            if (countflag == 296 && dataBuffer[294] == 0xFE && dataBuffer[295] == 0xFE)
                            {
                                ParsedQueue.Add(dataBuffer);
                                dataInCount++;
                                if(dataInCount %10000 ==0)
                                {
                                    Debug.WriteLine("Data in count is " + dataInCount);
                                }
                                dataBuffer = new byte[296];
                                countflag = 0;
                            }
                            else if (countflag == 296)
                            {
                                countflag = 0;
                                dataBuffer = new byte[296];
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

        private void MLPEAsync()
        {

        }

        


        }


}

   

        

