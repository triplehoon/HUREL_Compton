#include "Module.h"


using namespace std;

HUREL::Compton::Module::Module(eMouduleType moduleType, double(&eGain)[9], double(&mlpeGain)[9], std::string lutFileName, double moduleOffsetX = 0, double moduleOffsetY = 0, 	unsigned int binSize = SPECTRUM_ENERGY_BIN_SIZE, double maxEnergy = SPECTRUM_MAX_ENERGY) :
	mLutFileName(lutFileName),
	mModuleOffsetX(moduleOffsetX),
	mModuleOffsetY(moduleOffsetY),
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
		static_assert
	}

	
}

HUREL::Compton::Module::~Module()
{
}
