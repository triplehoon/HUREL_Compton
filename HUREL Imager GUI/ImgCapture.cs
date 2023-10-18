using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;

namespace HUREL_Imager_GUI
{
    //Spectrum : 980, 110, 630, 580
    //Spectrum + Isotopes list: 980, 110, 930, 580
    //Isotopes list : 1605, 110, 300, 580
    public class ImgCapture
    {
        private int _refX = 0;  //capture start point X
        public int refX { get; set; }

        private int _refY = 0;  //capture start point Y
        public int refY { get; set; }

        private int _imgW = 0;  //capture image width
        public int imgW { get; set; }

        private int _imgH = 0;  //capture image height
        public int imgH { get; set; }

        private string? filePath = null;    //image save path

        public ImgCapture(int refx, int refy, int refw, int refh)
        {
            refX = refx;
            refY = refy;
            imgW = refw;
            imgH = refh;
        }

        public void SetPath(string path)
        {
            filePath = path;
        }

        public void DoCaptureImage()
        {
            if (filePath != null)
            {
                if (imgW == 0 || imgH == 0)
                    return;

                FilePathCheck();

                using (Bitmap bitmap = new Bitmap(imgW, imgH))
                {
                    using (Graphics graphics = Graphics.FromImage(bitmap))
                    {
                        graphics.CopyFromScreen(refX, refY, 0, 0, bitmap.Size);
                    }

                    bitmap.Save(filePath, ImageFormat.Png);
                }
            }
        }

        //경로 확인 및 폴더 생성
        private void FilePathCheck()
        {
            string? directory = Path.GetDirectoryName(filePath);

            if (string.IsNullOrWhiteSpace(directory) == false)
            {
                if (Directory.Exists(directory) == false)
                {
                    Directory.CreateDirectory(directory);
                }
            }
        }
    }
}
