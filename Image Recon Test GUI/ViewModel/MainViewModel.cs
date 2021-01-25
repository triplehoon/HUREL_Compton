using GalaSoft.MvvmLight.Command;
using HelixToolkit.Wpf.SharpDX;
using HUREL.Compton;
using Microsoft.Win32;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace Image_Recon_Test_GUI.ViewModel
{
    public class MainViewModel : INotifyPropertyChanged
    {
        public MainViewModel()
        {
            SetImageSpace();
            AddAxisPoints();

            VMStatus = "VM Status";
        }

        BinaryToLMData binaryToLMData = new BinaryToLMData();


        private RelayCommand loadBinaryFileCommand;
        public ICommand LoadBinaryFileCommand
        {
            get { return (this.loadBinaryFileCommand) ?? (this.loadBinaryFileCommand = new RelayCommand(LoadBinaryFile)); }
        }
        private void LoadBinaryFile()
        {
            OpenFileDialog openFile = new OpenFileDialog();

            openFile.DefaultExt = ".bin";
            openFile.Filter = "Binary Files (*.bin)|*.bin";
            openFile.InitialDirectory = Directory.GetCurrentDirectory();

            Nullable<bool> result = openFile.ShowDialog();
            if(result == true) 
            {
                VMStatus = "Loading Binary File";
                Task loadingBF = Task.Run(()=>binaryToLMData.LoadBinaryFile(openFile.FileName));
                loadingBF.Wait();
                VMStatus = "Done Loading Binary File";
            }
            else
            {
                VMStatus = "Select Binary File";
                return;
            }

            
           
            
        }

        private PointGeometry3D reconPoint;
        public PointGeometry3D ReconPoint
        {
            get { return reconPoint; }
            set { reconPoint = value; OnPropertyChanged(nameof(ReconPoint)); }
        }

        private PointGeometry3D axisPoint;
        public PointGeometry3D AxisPoint
        {
            get { return axisPoint; }
            set { axisPoint = value; OnPropertyChanged(nameof(AxisPoint)); }
        }

        /// <summary>
        /// Add axis in positive, 0.1 m
        /// </summary>
        private void AddAxisPoints()
        {
            Vector3Collection vector3s = new Vector3Collection();
            Color4Collection color4s = new Color4Collection();
            vector3s.Add(new SharpDX.Vector3());
            color4s.Add(new SharpDX.Color4(0, 0, 0, 1));

            for (int i = 1; i < 20; i++)
            {
                vector3s.Add(new SharpDX.Vector3() { X = 0.1f * i });
                color4s.Add(new SharpDX.Color4(1,0,0,1));
                vector3s.Add(new SharpDX.Vector3() { Y = 0.1f * i });
                color4s.Add(new SharpDX.Color4(0, 1, 0, 1));
                vector3s.Add(new SharpDX.Vector3() { Z = 0.1f * i });
                color4s.Add(new SharpDX.Color4(0, 0, 1, 1));
            }
            

            AxisPoint = new PointGeometry3D() { Positions = vector3s, Colors = color4s };

        }

        private RelayCommand drawBPCommand;
        public ICommand DrawBPCommand
        {
            get { return (this.drawBPCommand) ?? (this.drawBPCommand = new RelayCommand(DrawBP)); }
        }

        private void DrawBP()
        {
            VMStatus = "Drawing BP";

            Task.Run(() =>
            {
                var bpData = ImageRecon.BPtoPointCloud(ImageSpace, binaryToLMData.LACC.ListedLMData, 5);
                var bpVectors = bpData.Item1;
                var bpColor4s = bpData.Item2;

                ReconPoint = new PointGeometry3D() { Positions = bpVectors, Colors = bpColor4s };
                VMStatus = "Drawing BP Done";
            });

            
        }


        private RelayCommand drawEMCommand;
        public ICommand DrawEMCommand
        {
            get { return (this.drawEMCommand) ?? (this.drawEMCommand = new RelayCommand(DrawEM)); }
        }

        private void DrawEM()
        {
            throw new NotImplementedException();
        }


        private RelayCommand drawSOECommand;
        public ICommand DrawSOECommand
        {
            get { return (this.drawSOECommand) ?? (this.drawSOECommand = new RelayCommand(DrawSOE)); }
        }

        private void DrawSOE()
        {
            throw new NotImplementedException();
        }




        #region Image Space Setting
        private Vector3Collection ImageSpace = new Vector3Collection();

        private float minX = -1;
        public float MinX
        {
            get { return minX; }
            set { minX = value; OnPropertyChanged(nameof(MinX)); }
        }

        private float maxX = 1;
        public float MaxX
        {
            get { return maxX; }
            set { maxX = value; OnPropertyChanged(nameof(MaxX)); }
        }


        private float minY = -1;
        public float MinY
        {
            get { return minY; }
            set { minY = value; OnPropertyChanged(nameof(MinY)); }
        }

        private float maxY = 1;
        public float MaxY
        {
            get { return maxY; }
            set { maxY = value; OnPropertyChanged(nameof(MaxY)); }
        }


        private float minZ = 0;
        public float MinZ
        {
            get { return minZ; }
            set { minZ = value; OnPropertyChanged(nameof(MinZ)); }
        }

        private float maxZ = 3;
        public float MaxZ
        {
            get { return maxZ; }
            set { maxZ = value; OnPropertyChanged(nameof(MaxZ)); }
        }

        private float voxelSize = 0.05f;
        public float VoxelSize
        {
            get { return voxelSize; }
            set { voxelSize = value; OnPropertyChanged(nameof(VoxelSize)); }
        }

        private RelayCommand setImageSpaceCommand;
        public ICommand SetImageSpaceCommand
        {
            get { return (this.setImageSpaceCommand) ?? (this.setImageSpaceCommand = new RelayCommand(SetImageSpace)); }
        }
        private void SetImageSpace()
        {
            VMStatus = "Image Space is Reset";
            ImageSpace = ImageRecon.GetImageSpaceByVoxel(MinX, MaxX, MinY, MaxY, MinZ, MaxZ, VoxelSize);
            VMStatus = "Image Space is Reset. Voxel Count: " + ImageSpace.Count;
        }

        #endregion


        private string vmStatus;
        public string VMStatus
        {
            get { return vmStatus; }
            set { vmStatus = value; OnPropertyChanged(nameof(VMStatus)); }
        }






        public event PropertyChangedEventHandler PropertyChanged;
        protected void OnPropertyChanged(string propertyName)
        {
            if (PropertyChanged != null)
            {
                PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
            }
        }
    }
}
