#include "Module.h"


using namespace std;
using namespace HUREL::Compton;
HUREL::Compton::Module::Module() :
    mModuleType(HUREL::Compton::eMouduleType::MONO),
    mModuleName(""),
    mModuleOffsetX(0),
    mModuleOffsetY(0),
    mModuleOffsetZ(0),
    mIsModuleSet(false),
    mEnergySpectrum(EnergySpectrum(5, 3000))
{
}

HUREL::Compton::Module::Module(eMouduleType moduleType,
    std::string configDir, std::string moduleName,
    double moduleOffsetX, double moduleOffsetY, double moduleOffsetZ,
    unsigned int binSize, double maxEnergy):
    mModuleType(moduleType),
    mModuleName(moduleName),
	mModuleOffsetX(moduleOffsetX),
	mModuleOffsetY(moduleOffsetY),
    mModuleOffsetZ(moduleOffsetZ),
	mIsModuleSet(false),
    mEnergySpectrum(EnergySpectrum(binSize, maxEnergy))
{
    std::string lutFileName = configDir + "\\LUT" + "\\" + moduleName + ".csv";
    std::string gainFileName = configDir + "\\Gain" + "\\" + moduleName + ".csv";
    
    if (moduleType == eMouduleType::QUAD)
    {
        double gain[10];
        if (LoadGain(gainFileName, eMouduleType::QUAD, gain))
        {
            for (int i = 0; i < 10; ++i)
            {
                mGain[i] = gain[i];
            }
            string msg = "Successfuly to load a gain file : " + moduleName;
            HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::Module", msg);
        }        
        else
        {
            string msg = "FAIL to load a gain file: " + moduleName;
            HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::Module", msg);
            mIsModuleSet = false;
            return;
        }
    }        	
	if (LoadLUT(lutFileName))
	{
		//cout << "Module.cpp: Successfuly to load a lut file: " << moduleName << endl;
        string msg = "Successfuly to load a lut file: " + moduleName;
        HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::Module", msg);
		mIsModuleSet = true;
	}
	else
	{
		//cout << "Module.cpp: FAIL to load a lut file: " << moduleName << endl;
        string msg = "FAIL to load a lut file: " + moduleName;
        HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::Module", msg);
		assert(false);
	}	

    assert(mModuleType != eMouduleType::MONO);
}

std::tuple<unsigned int, unsigned int> HUREL::Compton::Module::FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const double(&normalizePMTValue)[9]) const
{
    assert(minX < maxX);
    assert(minX < maxX);

    if (static_cast<int>(mLutSize) - 1 < maxX)
    {
        maxX = mLutSize - 1;
    }
    if (static_cast<int>(mLutSize) - 1 < maxY)
    {
        maxY = mLutSize - 1;
    }
    if (static_cast<int>(minX) < 0)
    {
        minX = 0;
    }
    if (static_cast<int>(minY) < 0)
    {
        minY = 0;
    }


    double valMaxChk = -numeric_limits<double>::max();
    std::tuple<unsigned int, unsigned int> maxPoint;
    for (unsigned int x = static_cast<unsigned int>(minX); x <= static_cast<unsigned int>(maxX); x += gridSize)
    {
        for (unsigned int y = static_cast<unsigned int>(minY); y <= static_cast<unsigned int>(maxY); y += gridSize)
        {
            double val = 0;
            if (isnan(mXYLogMue[x][y][0]))
            {
                continue;
            }
            for (int i = 0; i < 9; i++)
            {
               /* if (mXYLogMue[x][y] == NULL)
                {
                    continue;
                }*/
                val += mXYLogMue[x][y][i] * normalizePMTValue[i];

            }
            val += mXYSumMu[x][y];

            if (val > valMaxChk)
            {
                valMaxChk = val;
                get<0>(maxPoint) = x;
                get<1>(maxPoint) = y;
            }
        }
    }
    
    return maxPoint;
}

HUREL::Compton::Module::~Module()
{
    for (int i = 0; i < static_cast<int>(mLutSize); ++i)
    {
        for (int j = 0; j < static_cast<int>(mLutSize); ++j)
        {
            delete[] mXYLogMue[i][j];
        }
        delete[] mXYLogMue[i];
        delete[] mXYSumMu[i];         
    }
}

bool HUREL::Compton::Module::LoadLUT(std::string fileName)
{
    std::ifstream io;
    io.open(fileName.c_str());
    vector<std::array<double, 80>> lutData;
    lutData.reserve(100000);

    if (!io.is_open())
    {
        //cerr << "Cannot open a file" << endl;

        io.close();
        return false;
    }
    unsigned int lutSize = 0;
    while (!io.eof())
    {   
        string line;

        std::getline(io, line);

        /* for each value */
        string val;        
        std::stringstream lineStream(line);

        std::array<double, 80> row;

        int i = 0;
        while (std::getline(lineStream, val, ','))
        {
            row[i] = stod(val);
            ++lutSize;
            ++i;
        }                        
        
        lutData.push_back(row);
    }
    lutData.shrink_to_fit();
    mLutSize = static_cast<unsigned int>(sqrt(lutData.size()));
    
    //Memory alloc
    mXYLogMue = new double**[mLutSize];
    mXYSumMu = new double*[mLutSize];
    for (int i = 0; i < static_cast<int>(mLutSize); ++i)
    {
        mXYLogMue[i] = new double*[mLutSize];
        mXYSumMu[i] = new double[mLutSize];     
    }
    
    
    for (int i = 0; i < lutData.size(); ++i)
    {
        int x = static_cast<int>(lutData[i][0]);
        int y = static_cast<int>(lutData[i][1]);

        int indexOffset = (mLutSize - 1) / 2;

        assert((x + indexOffset) >= 0);

        unsigned int indexX = x + indexOffset;
        unsigned int indexY = y + indexOffset;
        mXYLogMue[indexX][indexY] = new double[9] {lutData[i][3], lutData[i][4], lutData[i][5],
        lutData[i][6],lutData[i][7],lutData[i][8],lutData[i][9],lutData[i][10],lutData[i][11] };
        mXYSumMu[indexX][indexY] = lutData[i][12];
    }

    //cout << "Done loading look up table" << endl;

    io.close();
    return true;
}

const bool HUREL::Compton::Module::IsModuleSet() const
{
    return mIsModuleSet;
}

bool HUREL::Compton::Module::LoadGain(std::string fileName, eMouduleType moduleType, double* outEGain)
{
    std::ifstream io;
    io.open(fileName);
    vector<vector<double>> lutData;

    assert(outEGain != NULL);
    if (io.fail())
    {
        //cout << "Cannot open a file" << endl;
        string msg = "Cannot open a file";
        HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::Module", msg);
        io.close();        
        return false;
    }

    unsigned int pmtCounts = 0;

    switch (moduleType)
    {
    case HUREL::Compton::eMouduleType::MONO:
        pmtCounts = 36;
        break;
    case HUREL::Compton::eMouduleType::QUAD:
        pmtCounts = 9;
        break;
    case HUREL::Compton::eMouduleType::QUAD_DUAL:
        pmtCounts = 9;
        break;
    default:
        pmtCounts = 0;
        return false;
        break;
    }

    while (!io.eof())
    {
        string line;

        std::getline(io, line);

        /* for each value */
        string val;
        vector<double> row;
        std::stringstream lineStream(line);
        unsigned int i = 0;
        while (std::getline(lineStream, val, ','))
        {       
            assert(i < pmtCounts + 1);
            outEGain[i] = stod(val);
            ++i;
        }      
    }
    io.close();

    return true;
}

const Eigen::Vector4d HUREL::Compton::Module::FastMLPosEstimation(const unsigned short pmtADCValue[]) const
{

    std::tuple<unsigned int, unsigned int> maxPoint;

    const double normalizedPMTValue[9] { 
    static_cast<double>(pmtADCValue[0]) * mGain[0],
    static_cast<double>(pmtADCValue[1]) * mGain[1],
    static_cast<double>(pmtADCValue[2]) * mGain[2],
    static_cast<double>(pmtADCValue[3]) * mGain[3],
    static_cast<double>(pmtADCValue[4]) * mGain[4],
    static_cast<double>(pmtADCValue[5]) * mGain[5],
    static_cast<double>(pmtADCValue[6]) * mGain[6],
    static_cast<double>(pmtADCValue[7]) * mGain[7],
    static_cast<double>(pmtADCValue[8]) * mGain[8]};
    
    int gridSize = mLutSize / 3;//45
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize, gridSize, mLutSize - 1, gridSize, mLutSize - 1, normalizedPMTValue);
    
    int gridSize2 = mLutSize / 9;//14
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize , get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);
    
    int gridSize3 = mLutSize / 27 ;//3
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize3, get<0>(maxPoint) - gridSize2, get<0>(maxPoint) + gridSize2, get<1>(maxPoint) - gridSize2, get<1>(maxPoint) + gridSize2, normalizedPMTValue);
    



    int gridSize4 = 1;
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize4, get<0>(maxPoint) - gridSize3 , get<0>(maxPoint) + gridSize3 , get<1>(maxPoint) - gridSize3,  get<1>(maxPoint) + gridSize3, normalizedPMTValue);

    ////int gridSize5 = 1;
    //maxPoint = FastMLPosEstimationFindMaxIndex(gridSize5, get<0>(maxPoint) - gridSize4 - 5, get<0>(maxPoint) + gridSize4 + 5, get<1>(maxPoint) - gridSize4 - 5, get<1>(maxPoint) + gridSize4 + 5, normalizedPMTValue);

    Eigen::Vector4d point;

    point[0] = (static_cast<double>(get<0>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetX;
    point[1] = (static_cast<double>(get<1>(maxPoint)) - static_cast<double>(mLutSize - 1) / 2.0) / 1000 + mModuleOffsetY;
    point[2] = mModuleOffsetZ;
    point[3] = 1;
    return point;   
}

const Eigen::Vector4d HUREL::Compton::Module::FastMLPosEstimation(unsigned short(&pmtADCValue)[36]) const
{
    assert(false && "FastMLPosEstimation NOT IMplented");
    return Eigen::Vector4d(0,0,0, 1);
}

const double HUREL::Compton::Module::GetEcal(const unsigned short pmtADCValue[]) const
{
    double sumEnergy = 0;
    unsigned short checkZero = 0;
    for (int i = 0; i < 9; ++i)
    {
        sumEnergy += static_cast<double>(pmtADCValue[i]) * mGain[i];
        checkZero += pmtADCValue[i];
    }
    if (checkZero == 0)
    {
        return static_cast<double>(NAN);
    }
    sumEnergy += mGain[9];
    return mEnergyCalibrationA * sumEnergy * sumEnergy + mEnergyCalibrationB * sumEnergy + mEnergyCalibrationC;
}

void HUREL::Compton::Module::SetEnergyCalibration(double a, double b, double c)
{
    mEnergyCalibrationA = a;
    mEnergyCalibrationB = b;
    mEnergyCalibrationC = c;
}

std::tuple<double, double, double>  HUREL::Compton::Module::GetEnergyCalibration()
{
    return make_tuple(mEnergyCalibrationA,  mEnergyCalibrationB, mEnergyCalibrationC);
}

EnergySpectrum& HUREL::Compton::Module::GetEnergySpectrum()
{
    return mEnergySpectrum;
}

const std::string HUREL::Compton::Module::GetModuleName() const
{
    return mModuleName;
}

bool HUREL::Compton::Module::SetGain(eMouduleType type, std::vector<double> gain)
{
    if (type != eMouduleType::QUAD)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < 10; ++i)
        {
            mGain[i] = gain[i];
        }
    }
}