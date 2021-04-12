﻿
using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using AsyncAwaitBestPractices;
using AsyncAwaitBestPractices.MVVM;
using HUREL.Compton;


namespace Compton_GUI_WPF.ViewModel
{
    public partial class MainViewModel : INotifyPropertyChanged
    {
        #region Variables
        private bool isFPGAOn;
        public bool IsFPGAOn
        {
            get
            {
                return isFPGAOn;
            }
            set
            {
                isFPGAOn = value;
                OnPropertyChanged(nameof(IsFPGAOn));
            }
        }

        private double batteryVoltage;
        public double BatteryVoltage
        {
            get
            {
                return batteryVoltage;
            }
            set
            {
                batteryVoltage = value;
                OnPropertyChanged(nameof(BatteryVoltage));
            }
        }

        private bool isHvModuleOn;
        public bool IsHvModuleOn
        {
            get
            {
                return isHvModuleOn;
            }
            set
            {
                isHvModuleOn = value;
                OnPropertyChanged(nameof(IsHvModuleOn));
            }
        }
        private double hvModuleVoltage;
        public double HvModuleVoltage
        {
            get
            {
                return hvModuleVoltage;
            }
            set
            {
                hvModuleVoltage = value;
                OnPropertyChanged(nameof(HvModuleVoltage));
            }
        }

        private double hvModuleCurrent;
        public double HvModuleCurrent
        {
            get
            {
                return hvModuleCurrent;
            }
            set
            {
                hvModuleCurrent = value;
                OnPropertyChanged(nameof(HvModuleCurrent));
            }
        }

        private ObservableCollection<string> comPorts = new ObservableCollection<string>(LaccControl.PortsName);
        public ObservableCollection<string> ComPorts
        {
            get
            {
                return comPorts;
            }
            set
            {
                comPorts = value;
                OnPropertyChanged(nameof(ComPorts));
            }
        }

        private bool isSerialOpen;
        public bool IsSerialOpen
        {
            get
            {
                return isSerialOpen;
            }
            set
            {
                isSerialOpen = value;
                OnPropertyChanged(nameof(IsSerialOpen));
            }
        }

        private string selectedComPort;
        public string SelectedComport
        {
            get
            {
                return selectedComPort;
            }
            set
            {
                selectedComPort = value;
                LaccControl.SelectedPortName = value;
                OnPropertyChanged(nameof(SelectedComport));
            }
        }

        private bool isSwitch1_On;
        public bool IsSwitch1_On
        {
            get
            {
                return isSwitch1_On;
            }
            set
            {
                isSwitch1_On = value;
                OnPropertyChanged(nameof(IsSwitch1_On));
            }
        }
        private bool isSwitch2_On;
        public bool IsSwitch2_On
        {
            get
            {
                return isSwitch2_On;
            }
            set
            {
                isSwitch2_On = value;
                OnPropertyChanged(nameof(IsSwitch2_On));
            }
        }
        private bool isSwitch3_On;
        public bool IsSwitch3_On
        {
            get
            {
                return isSwitch3_On;
            }
            set
            {
                isSwitch3_On = value;
                OnPropertyChanged(nameof(IsSwitch3_On));
            }
        }
        private bool isSwitch4_On;
        public bool IsSwitch4_On
        {
            get
            {
                return isSwitch4_On;
            }
            set
            {
                isSwitch4_On = value;
                OnPropertyChanged(nameof(IsSwitch4_On));
            }
        }
        private bool isSwitch5_On;
        public bool IsSwitch5_On
        {
            get
            {
                return isSwitch5_On;
            }
            set
            {
                isSwitch5_On = value;
                OnPropertyChanged(nameof(IsSwitch5_On));
            }
        }
        private bool isSwitch6_On;
        public bool IsSwitch6_On
        {
            get
            {
                return isSwitch6_On;
            }
            set
            {
                isSwitch6_On = value;
                OnPropertyChanged(nameof(IsSwitch6_On));
            }
        }

        #endregion

        private AsyncCommand updatePortsNameCommand;
        public IAsyncCommand UpdatePortsNameCommand
        {
            get { return updatePortsNameCommand ?? (updatePortsNameCommand = new AsyncCommand(UpdataePorts)); }
        }

        private async Task UpdataePorts()
        {
            await Task.Run(()=>LaccControl.UpdatePortsName());
            ComPorts = new ObservableCollection<string>(LaccControl.PortsName);
            SelectedComport = LaccControl.SelectedPortName;
        }
        private AsyncCommand startCommunicationCommand;
        public IAsyncCommand StartCommunicationCommand
        {
            get { return startCommunicationCommand ?? (startCommunicationCommand = new AsyncCommand(StartCommunication)); }
        }

        private async Task StartCommunication()
        {

            await Task.Run(() =>
            {
                bool check = false;
                check = LaccControl.StartCommunication();

                if (check)
                {
                    IsSerialOpen = true;
                    UpdateAllParams();
                }
                else
                {
                    IsSerialOpen = false;
                }
                
            });

            
        }

        private AsyncCommand stopCommunicationCommand;
        public IAsyncCommand StopCommunicationCommand
        {
            get { return stopCommunicationCommand ?? (stopCommunicationCommand = new AsyncCommand(StopCommunication)); }
        }

        private async Task StopCommunication()
        {

            await Task.Run(() =>
            {
                LaccControl.StopCommunication();
                 IsSerialOpen = false;
            });


        }
        private void UpdateAllParams()
        {
            LaccControl.CheckParams();
            IsFPGAOn = LaccControl.IsFPGAOn;
            HvModuleVoltage = LaccControl.HvModuleVoltage;
            if (HvModuleVoltage > 500)
            {
                IsHvModuleOn = true;
            }
            HvModuleCurrent = LaccControl.HvModuleCurrent;
            BatteryVoltage = LaccControl.BatteryVoltage;
            IsSwitch1_On = LaccControl.IsSwitchOn[0];
            IsSwitch2_On = LaccControl.IsSwitchOn[1];
            IsSwitch3_On = LaccControl.IsSwitchOn[2];
            IsSwitch4_On = LaccControl.IsSwitchOn[3];
            IsSwitch5_On = LaccControl.IsSwitchOn[4];
            IsSwitch6_On = LaccControl.IsSwitchOn[5];
        }
        

        private AsyncCommand startHvModuleCommand;
        public IAsyncCommand StartHvModuleCommand
        {
            get
            {
                return startHvModuleCommand ?? (startHvModuleCommand = new AsyncCommand(StartHvModule));
            }
        }
        private async Task StartHvModule ()
        {
            IsUpdateHvRunning = true;
            UpdateHv().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            await Task.Run(() =>
            {
                try
                {
                    LaccControl.SetHvMoudle(true);
                    IsHvModuleOn = true;
                }
                catch (TimeoutException e)
                {
                    Trace.WriteLine(e.Message);
                    StopCommunicationCommand.Execute(null);
                    IsSerialOpen = false;
                    IsHvModuleOn = false;
                }
                finally
                {
                    IsUpdateHvRunning = false;
                }


                UpdateAllParams();
            });


        }


        private AsyncCommand stopHvModuleCommand;
        public IAsyncCommand StopHvModuleCommand
        {
            get
            {
                return stopHvModuleCommand ?? (stopHvModuleCommand = new AsyncCommand(StopHvModule));
            }
        }
        private async Task StopHvModule()
        {
            IsUpdateHvRunning = true;
            UpdateHv().SafeFireAndForget(onException: ex => Debug.WriteLine(ex));
            await Task.Run(() =>
            {
                try 
                {
                    LaccControl.SetHvMoudle(false);
                    IsHvModuleOn = false;
                }
                catch (TimeoutException e)
                {
                    Trace.WriteLine(e.Message);
                    StopCommunicationCommand.Execute(null);
                    IsSerialOpen = false;
                }
                finally
                {
                    IsUpdateHvRunning = false;
                    IsHvModuleOn = false;
                }

                UpdateAllParams();
            });
        }

        private bool isUpdateHvRunning = false;
        public bool IsUpdateHvRunning {
            get 
            { 
                return isUpdateHvRunning; 
            }
            set 
            { 
                isUpdateHvRunning = value;
                OnPropertyChanged(nameof(IsUpdateHvRunning));
            }
        }
        private async Task UpdateHv()
        {
            await Task.Run(() =>
            {
                while (IsUpdateHvRunning)
                {
                    HvModuleVoltage = LaccControl.HvModuleVoltage;
                    Thread.Sleep(1);
                }

            });
        }


        private AsyncCommand startFpgaCommand;
        public IAsyncCommand StartFpgaCommand
        {
            get { return startFpgaCommand ?? (startFpgaCommand = new AsyncCommand(StartFpga)); }
        }
        private async Task StartFpga()
        {
            await Task.Run(() =>
            {
                try
                {
                    LaccControl.SetFPGA(true);
                }
                catch(TimeoutException e)
                {
                    Trace.WriteLine(e.Message);
                    StopCommunicationCommand.Execute(null);
                    IsSerialOpen = false;
                }
                
                UpdateAllParams();                
            });
        }


        private AsyncCommand stopFpgaCommand;
        public IAsyncCommand StopFpgaCommand
        {
            get { return stopFpgaCommand ?? (stopFpgaCommand = new AsyncCommand(StopFpga)); }
        }
        private async Task StopFpga()
        {
            await Task.Run(() =>
            {
                try
                {
                    LaccControl.SetFPGA(false);
                }
                catch (TimeoutException e)
                {
                    Trace.WriteLine(e.Message);
                    StopCommunicationCommand.Execute(null);
                    IsSerialOpen = false;
                }

                UpdateAllParams();
            });
        }


    }







}
