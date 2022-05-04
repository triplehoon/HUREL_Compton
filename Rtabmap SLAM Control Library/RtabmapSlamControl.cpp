#include "RtabmapSlamControl.h"

using namespace HUREL::Compton;

bool HUREL::Compton::RtabmapSlamControl::Initiate(std::string* outMessage)
{
	return false;
}

RtabmapSlamControl& HUREL::Compton::RtabmapSlamControl::instance()
{
	static RtabmapSlamControl* instance = new RtabmapSlamControl();
	//std::cout << "instance: " << instance << std::endl;
	return *instance;
}
