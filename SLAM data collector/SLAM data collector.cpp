// SLAM data collector.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <chrono>
#include <vector>

#include "RtabmapSlamControl.h"

using namespace HUREL::Compton;
using namespace std;

string PrintEigenInOneLine(Eigen::Matrix4d);

int main()
{
    std::cout << "Hello World!\n";

    HUREL::Compton::RtabmapSlamControl& SlamControl = HUREL::Compton::RtabmapSlamControl::instance();

    string outMessage;
    SlamControl.Initiate(&outMessage);

    std::cout << outMessage;

    SlamControl.StartVideoStream();
    SlamControl.StartSlamPipe();
    chrono::system_clock::time_point StartTime = chrono::system_clock::now();
    std::vector<Eigen::Matrix4d> poses;
    std::vector<chrono::system_clock::time_point> timePoints;
    while (true)
    {
        if (cv::waitKey(95) == 27)
        {
            break;
        }
        cv::Mat img = SlamControl.GetCurrentVideoFrame();
        cv::imshow("img", img);        
        poses.push_back(SlamControl.GetOdomentry());
        timePoints.push_back(chrono::system_clock::now());
       
    }


    //save point Cloud and pose data.
    std::string FileName = "test";
    Eigen::Matrix4d t265toLACCPosTransform;
    t265toLACCPosTransform << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
        0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
        0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
        0, 0, 0, 1;
    
    ofstream fon;

    fon.open(FileName + ".csv");
    for (int i = 0; i < poses.size(); ++i)
    {
        string poseString = PrintEigenInOneLine(poses[i] * t265toLACCPosTransform);
        chrono::milliseconds mill = chrono::duration_cast<chrono::milliseconds>(timePoints[i] - StartTime);
       
        std::cout << mill.count() << "," << poseString << std::endl;
        string outMsg = to_string(mill.count()) + "," + poseString + "\n";
        fon.write(outMsg.c_str(), outMsg.size());
    }
    fon.close();


    open3d::geometry::PointCloud pc = SlamControl.GetSlamPointCloud();
    open3d::io::WritePointCloudOption option;
    open3d::io::WritePointCloudToPLY(FileName + ".ply", pc, option);
    

    SlamControl.StopSlamPipe();
    SlamControl.StopVideoStream();
}

string PrintEigenInOneLine(Eigen::Matrix4d m)
{
    string data;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            data += std::to_string(m(i, j));
            if (i == 3 && j == 3)
            {
                break;
            }
            data += ",";
        }        
    }
    return data;
}

