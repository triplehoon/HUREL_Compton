using Python.Runtime;
using System;
using System.Collections.Generic;
using System.IO;
using System.Text;

namespace Tes_pythonnet
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");
            //load Spectrum
            List<float> listModeData = new List<float>();
            int Size = 0;

            using (FileStream f = File.Open(@"E:\OneDrive - 한양대학교\01.Hurel\01.현재작업\20210923 핵종분석\NASA-gamma\examples\Cs137_0.01uSv_930s_ListModeData.csv", FileMode.Open))
            {
                using (StreamReader sr = new StreamReader(f, Encoding.UTF8, false))
                {
                    while (sr.ReadLine() != null)
                    {
                        ++Size;
                    }
                }
            }
            using (FileStream f = File.Open(@"E:\OneDrive - 한양대학교\01.Hurel\01.현재작업\20210923 핵종분석\NASA-gamma\examples\Cs137_0.01uSv_930s_ListModeData.csv", FileMode.Open))
            {
                using (StreamReader sr = new StreamReader(f, Encoding.UTF8, false))
                {
                    string strLineValue = null;
                    string[] values = null;
                    int lineNum = 0;
                    Console.Write("Loading... ");
                    var progress = new ProgressBar();
                    while ((strLineValue = sr.ReadLine()) != null)
                    {
                        // Must not be empty.
                        if (string.IsNullOrEmpty(strLineValue))
                        {
                            break;
                        }

                        values = strLineValue.Split(',');

                        float scatterE = Convert.ToSingle(values[3]);
                        float absorberE = Convert.ToSingle(values[7]);

                        if (!float.IsNaN(scatterE) && !float.IsNaN(absorberE))
                        {
                            listModeData.Add(scatterE + absorberE);
                        }
                        else if (!float.IsNaN(scatterE))
                        {
                            listModeData.Add(scatterE);
                        }
                        else if (!float.IsNaN(absorberE))
                        {
                            listModeData.Add(absorberE);
                        }

                        ++lineNum;
                        progress.Report((double)lineNum / Size);
                    }
                    progress.Dispose();
                }
            }
            Console.WriteLine("Done!");



            List<double> PeakE = new List<double>();

            using (Py.GIL())
            {
                try
                {
                    dynamic import = Py.Import("blah");
                }
                catch(Exception e)
                {
                    
                    Console.WriteLine(e.Message);
                }
                
                dynamic np = Py.Import("numpy");
                dynamic nasagamma = Py.Import("nasagamma");
                dynamic sp = nasagamma.spectrum;
                dynamic ps = nasagamma.peaksearch;

                dynamic eData = np.array(listModeData);
                dynamic bin = np.arange(0, 3000, 5);
                dynamic hist = np.histogram(eData, bin);
                dynamic cts_np = hist[0];
                bin = hist[1];
                dynamic erg = np.resize(bin, (np.size(bin) - 1));

                dynamic spect = sp.Spectrum(cts_np, null, erg, "keV");

                dynamic fwhm_at_0 = 0.1;
                dynamic ref_fwhm = 50;
                dynamic ref_x = 662;
                dynamic min_snr = 3.0;
                // instantiate a peaksearch object
                dynamic search = ps.PeakSearch(spect, ref_x, ref_fwhm, fwhm_at_0, min_snr);
                dynamic peakIdx = search.peaks_idx;

                for (int i = 0; i < (int)np.size(peakIdx); ++i)
                {
                    PeakE.Add((int)erg[peakIdx[i]]);
                }
                
                Console.Write("Peaks: ");
                
                foreach (var p in PeakE)
                {
                    Console.Write($"{p}, ");
                }

                Console.WriteLine();
                search.plot_components();
            }

        }
    }
}
