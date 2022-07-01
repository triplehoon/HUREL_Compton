// SLAM data collector.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>


#include "RtabmapSlamControl.h"

using namespace HUREL::Compton;
using namespace std;

int main()
{
    std::cout << "Hello World!\n";

    HUREL::Compton::RtabmapSlamControl& SlamControl = HUREL::Compton::RtabmapSlamControl::instance();

    string outMessage;
    SlamControl.Initiate(&outMessage);

    std::cout << outMessage;

    SlamControl.StartVideoStream();
    SlamControl.StartSlamPipe();
    while (cv::waitKey(100))
    {

        cv::Mat img = SlamControl.GetCurrentVideoFrame();
        cv::imshow("img", img);
        Eigen::Matrix4d odo = SlamControl.GetOdomentry();
        std::cout << odo;

    }


}

