#include "Module.h"


using namespace std;

HUREL::Compton::Module::Module(eMouduleType moduleType, double(&eGain)[9], double(&mlpeGain)[9], std::string lutFileName, double moduleOffsetX = 0, double moduleOffsetY = 0, double moduleOffsetZ = 0, unsigned int binSize = SPECTRUM_ENERGY_BIN_SIZE, double maxEnergy = SPECTRUM_MAX_ENERGY) :
    mModuleType(moduleType),
    mLutFileName(lutFileName),
	mModuleOffsetX(moduleOffsetX),
	mModuleOffsetY(moduleOffsetY),
    mModuleOffsetZ(moduleOffsetZ),
	mIsModuleSet(false)
{
	mEnergySpectrum = EnergySpectrum(binSize, maxEnergy);
	for (int i = 0; i < 9; ++i)
	{
		mEnergyGain[i] = eGain[i];
		mMlpeGain[i] = mlpeGain[i];
	}
	if (LoadLUT(mLutFileName))
	{
		cout << "Successfuly to load a lut file" << endl;
		mIsModuleSet = true;
	}
	else
	{
		cout << "FAIL to load a lut file" << endl;
		assert(false);
	}	

    assert(mModuleType != eMouduleType::MONO);
}

std::tuple<unsigned int, unsigned int> HUREL::Compton::Module::FastMLPosEstimationFindMaxIndex(const unsigned int gridSize, int minX, int maxX, int minY, int maxY, const double(&normalizePMTValue)[9]) const
{
    assert(minX < maxX);
    assert(minX < maxX);

    if (mLutSize - 1 < maxX)
    {
        maxX = mLutSize - 1;
    }
    if (mLutSize - 1 < maxY)
    {
        maxY = mLutSize - 1;
    }
    if (minX < 0)
    {
        minX = 0;
    }
    if (minY < 0)
    {
        minY = 0;
    }


    double valMaxChk = numeric_limits<double>::min();
    std::tuple<unsigned int, unsigned int> maxPoint;
    for (unsigned int x = minX; x < maxX; minX += gridSize)
    {
        for (unsigned int y = minY; y < maxY; minY += gridSize)
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
            val -= mXYSumMu[x][y];

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

    for (int i = 0; i < mLutSize; ++i)
    {
        for (int j = 0; j < mLutSize; ++j)
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
    io.open(fileName);
    vector<vector<double>> lutData;

    if (io.fail())
    {
        cout << "Cannot open a file" << endl;

        io.close();
        return false;
    }

    while (io.eof())
    {   
        string line;

        std::getline(io, line);

        /* for each value */
        string val;
        vector<double> row;
        std::stringstream lineStream(line);
        while (std::getline(lineStream, val, ','))
        {
            row.push_back(stod(val));
        }                        
        lutData.push_back(row);
    }

    mLutSize = static_cast<unsigned int>(sqrt(lutData.size()));
    
    //Memory alloc
    mXYLogMue = new double**[mLutSize];
    mXYSumMu = new double*[mLutSize];
    for (int i = 0; i < mLutSize; ++i)
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

    cout << "Done loading look up table" << endl;

    io.close();
    return true;
}

const bool HUREL::Compton::Module::IsModuleSet() const
{
    return mIsModuleSet;
}

void HUREL::Compton::Module::LoadGain(std::string fileName, eMouduleType moduleType, double* outEGain)
{
    std::ifstream io;
    io.open(fileName);
    vector<vector<double>> lutData;

    assert(outEGain != NULL);
    if (io.fail())
    {
        cout << "Cannot open a file" << endl;

        io.close();        
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
        break;
    }

    while (io.eof())
    {
        string line;

        std::getline(io, line);

        /* for each value */
        string val;
        vector<double> row;
        std::stringstream lineStream(line);
        int i = 0;
        while (std::getline(lineStream, val, ','))
        {       
            assert(i < pmtCounts);
            outEGain[i] = stod(val);
        }      
    }
}

const Eigen::Vector3d HUREL::Compton::Module::FastMLPosEstimation(unsigned short(&pmtADCValue)[9]) const
{

    std::tuple<unsigned int, unsigned int> maxPoint;

    const double normalizedPMTValue[9] { 
    static_cast<double>(pmtADCValue[0]) * mEnergyGain[0],
    static_cast<double>(pmtADCValue[1]) * mEnergyGain[1],
    static_cast<double>(pmtADCValue[2]) * mEnergyGain[2],
    static_cast<double>(pmtADCValue[3]) * mEnergyGain[3],
    static_cast<double>(pmtADCValue[4]) * mEnergyGain[4],
    static_cast<double>(pmtADCValue[5]) * mEnergyGain[5],
    static_cast<double>(pmtADCValue[6]) * mEnergyGain[6],
    static_cast<double>(pmtADCValue[7]) * mEnergyGain[7],
    static_cast<double>(pmtADCValue[8]) * mEnergyGain[8]};


    int gridSize = mLutSize / 10;
    int gridSize2 = mLutSize / 20;
    int gridSize3 = 1;

    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize, gridSize, mLutSize - 1, gridSize, mLutSize - 1, normalizedPMTValue);
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize2, get<0>(maxPoint) - gridSize, get<0>(maxPoint) + gridSize, get<1>(maxPoint) - gridSize, get<1>(maxPoint) + gridSize, normalizedPMTValue);
    maxPoint = FastMLPosEstimationFindMaxIndex(gridSize3, get<0>(maxPoint) - gridSize2, get<0>(maxPoint) + gridSize2, get<1>(maxPoint) - gridSize2, get<1>(maxPoint) + gridSize2, normalizedPMTValue);
    Eigen::Vector3d point;
    point[0] = (static_cast<double>(get<0>(maxPoint)) - static_cast<double>(mLutSize) / 2) / 1000 + mModuleOffsetX;
    point[1] = (static_cast<double>(get<1>(maxPoint)) - static_cast<double>(mLutSize) / 2) / 1000 + mModuleOffsetY;
    point[2] = mModuleOffsetZ;
    return point;   
}

const Eigen::Vector3d HUREL::Compton::Module::FastMLPosEstimation(unsigned short(&pmtADCValue)[36]) const
{
    assert(false && "FastMLPosEstimation NOT IMplented");
}

const double HUREL::Compton::Module::GetEcal(unsigned short(&pmtADCValue)[9]) const
{
    double sumEnergy = 0;
    for (int i = 0; i < 9; ++i)
    {
        sumEnergy += static_cast<double>(pmtADCValue[i]) * mEnergyGain[i];
    }

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