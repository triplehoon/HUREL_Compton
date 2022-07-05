// Test Image reconstruction.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <thread>
#include <vector>
#include <time.h>

#include <open3d/geometry/LineSet.h>
#include <open3d/geometry/Image.h>
#include <open3d/geometry/RGBDImage.h>
#include <open3d/core/Tensor.h>
#include <open3d/core/CoreUtil.h>
#include <open3d/io/ImageIO.h>
#include <open3d/io/sensor/RGBDSensor.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <open3d/visualization/utility/Draw.h>
#include <open3d/pipelines/odometry/Odometry.h>
#include <open3d/pipelines/integration/ScalableTSDFVolume.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>

#include "RealsenseControl.h"
#include "LahgiControl.h"
#include "CodeMaskCalc.h"
#include "RadiationImage.h"

using namespace cv;
using namespace HUREL::Compton;
static Mat CodedMaskMat();
Mat MakeDetectorResponse(std::vector<ListModeData> lmData);
void ShowCV_32SAsJet(Mat img, int size);
void RemapAsSherical(const Mat& source, Mat& target);

std::tuple<bool, Eigen::Matrix4d_u, Eigen::Matrix6d> PairwayRgbdRegisteration(open3d::geometry::RGBDImage& source, open3d::geometry::RGBDImage& target);

#include <stdio.h>

int main()
{
    LahgiControl lahgi = LahgiControl::instance();
    lahgi.SetType(eMouduleType::TEST);
    lahgi.LoadListedListModeData("E:\\OneDrive - 한양대학교\\바탕 화면\\CC,CA code\\Cs137_0_0_1_test7_LMData.csv");

    std::vector<ListModeData> lmData = lahgi.GetListedListModeData();

    for (size_t i = 0; i < lmData.size(); ++i)
    {
        lmData[i].Scatter.RelativeInteractionPoint[0] -= T265_TO_LAHGI_OFFSET_X;
        lmData[i].Scatter.RelativeInteractionPoint[1] -= T265_TO_LAHGI_OFFSET_Y;
        lmData[i].Scatter.RelativeInteractionPoint[2] -= T265_TO_LAHGI_OFFSET_Z;
        lmData[i].Absorber.RelativeInteractionPoint[0] -= T265_TO_LAHGI_OFFSET_X;
        lmData[i].Absorber.RelativeInteractionPoint[1] -= T265_TO_LAHGI_OFFSET_Y;
        lmData[i].Absorber.RelativeInteractionPoint[2] -= T265_TO_LAHGI_OFFSET_Z;
    }

    RadiationImage radImg(lmData);
    ShowCV_32SAsJet(radImg.mDetectorResponseImage, 1000);
    ShowCV_32SAsJet(radImg.mCodedImage, 1000);
    
    ShowCV_32SAsJet(radImg.mComptonImage, 1000);
    ShowCV_32SAsJet(radImg.mHybridImage, 1000);

    return 0;
}
Mat MakeDetectorResponse(std::vector<ListModeData> lmData)
{
    double dectorWidth = 0.35;
    double pixelSize = 0.005;
    int pixelCount = static_cast<int>(round(dectorWidth / pixelSize));
    Mat img(pixelCount, pixelCount, CV_32S, Scalar(0));
    int* ptrImg = img.ptr<int>();
    for (ListModeData lm : lmData)
    {
        int i = 0;
        if (lm.Scatter.InteractionEnergy <= 620 || lm.Scatter.InteractionEnergy >= 700)
        {
            continue;
        }
        for (double x = -dectorWidth / 2; x < dectorWidth / 2; x += pixelSize)
        {
            int j = 0;
            for (double y = -dectorWidth / 2; y < dectorWidth / 2; y += pixelSize)
            {
                if (lm.Scatter.RelativeInteractionPoint[0] < x + pixelSize &&
                    lm.Scatter.RelativeInteractionPoint[1] < y + pixelSize &&
                    lm.Scatter.RelativeInteractionPoint[0] >= x &&
                    lm.Scatter.RelativeInteractionPoint[1] >= y)
                {
                    //img.at<int>(j, i)++;
                    ptrImg[j * pixelCount + i]++;
                }
                ++j;
            }
            ++i;
        }
    }    
    return img;
}
