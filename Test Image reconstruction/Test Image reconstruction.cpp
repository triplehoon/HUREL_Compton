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




//int main2()
//{
//    Camera* cmera = 0;
//
//}
//
//
//int main3()
//{
//    RealsenseControl rs = RealsenseControl::instance();
//    
//    std::string msg;
//    rs.InitiateRealsense(&msg);
//    std::cout << msg <<std::endl;
//    rs.IsPipeLineOn = true;
//    
//    std::thread t([&] {rs.RealsensesPipeline(); });
//
//    std::vector<open3d::geometry::RGBDImage> rgbdImgs;
//    while (true)
//    {              
//
//        Mat depth = rs.GetCurrentDepthFrame();
//
//        if (depth.cols < 1)
//        {
//            Sleep(1000);
//
//            continue;
//        }
//
//        open3d::geometry::Image depthImage;
//        
//        depthImage.Prepare(depth.cols, depth.rows, 1, 4);
//        std::cout << depth.cols << ", " << depth.rows << std::endl;
//        for (size_t i = 0; i < depthImage.data_.size(); ++i)
//        {
//            depthImage.data_[i] = depth.data[i];
//        }
//      
//        Mat colorBGRResize = rs.GetCurrentVideoFrame();
//        if (colorBGRResize.cols < 1)
//        {
//            continue;
//        }
//        
//        Mat colorBGR;
//        cv::resize(colorBGRResize, colorBGR, Size(depth.cols, depth.rows), 0, 0, INTER_NEAREST_EXACT);
//
//        Mat color;
//        cv::cvtColor(colorBGR, color, COLOR_BGR2RGB);
//        open3d::geometry::Image colorImage;
//        colorImage.Prepare(color.cols, color.rows, 3, 1);
//        std::cout << color.cols << ", " << color.rows << std::endl;
//        for (size_t i = 0; i < colorImage.data_.size(); ++i)
//        {
//            colorImage.data_[i] = color.data[i];
//        }
//       
//        open3d::geometry::RGBDImage rgbd(colorImage, depthImage);
//        rgbdImgs.push_back(rgbd);
//        Sleep(100);
//        if (rgbdImgs.size() == 20)
//        {
//            break;
//        }
//        auto image_ptr = std::make_shared<open3d::geometry::RGBDImage>();
//        *image_ptr = rgbd;
//        open3d::visualization::DrawGeometries({ image_ptr });
//    }
//    std::cout << "End data\n";
//    auto volume = open3d::pipelines::integration::ScalableTSDFVolume(0.02, 0.015, open3d::pipelines::integration::TSDFVolumeColorType::RGB8, 16, 4);
//    open3d::camera::PinholeCameraIntrinsic intrinsic(rgbdImgs[0].color_.width_, rgbdImgs[0].color_.height_, 418.131, 417.802, 419.79, 235.873);
//    volume.Integrate(rgbdImgs[0], intrinsic, Eigen::Matrix4d_u::Identity());
//    Eigen::Matrix4d_u odometry = Eigen::Matrix4d_u::Identity();
//    open3d::geometry::LineSet line;
//    std::shared_ptr <open3d::geometry::LineSet> lineptr = std::make_shared<open3d::geometry::LineSet>(line);
//    for (int i = 0; i < rgbdImgs.size() - 1; ++i)
//    {
//        clock_t start, end1, end2;
//        start = clock();
//        auto pair = PairwayRgbdRegisteration(rgbdImgs[i], rgbdImgs[i + 1]);
//        end1 = clock();
//        std::cout << "Pairway: " << (double)(end1 - start) << " ms\n";
//        Eigen::Matrix4d_u diff = std::get<1>(pair);
//        odometry = diff * odometry;
//        if (true)//std::get<0>(pair))
//        {
//            std::cout << odometry << std::endl;
//        }
//        else
//        {
//            std::cout << "fail" << std::endl;
//        }
//        auto odoInverse = odometry;    
//        start = clock();
//        volume.Integrate(rgbdImgs[i + 1], intrinsic, odoInverse);
//        end2 = clock();
//        std::cout << "Integrate: " << (double)(end2 - start) << " ms\n";
//    }
//    auto pc = volume.ExtractPointCloud();
//    Eigen::Vector3d lookAt(0, 0, 0);
//    Eigen::Vector3d up(0, -1, 0);
//    Eigen::Vector3d front(0, 0, -1);
//    double zoom = 0.02;
//    
//    open3d::visualization::DrawGeometries({ pc, lineptr }, "Test", 640 * 2, 2 * 480, 50, 50, false, false, false);// , & lookAt, & up, & front, & zoom);
//    rs.IsPipeLineOn = false;
//    t.join();
//    return 0;
//}
//std::tuple<bool, Eigen::Matrix4d_u, Eigen::Matrix6d> PairwayRgbdRegisteration(open3d::geometry::RGBDImage& source, open3d::geometry::RGBDImage& target)
//{        
//    open3d::camera::PinholeCameraIntrinsic intrinsic(source.color_.width_, source.color_.height_, 418.131, 417.802, 419.79, 235.873);
//    return open3d::pipelines::odometry::ComputeRGBDOdometry(source, target, intrinsic, Eigen::Matrix4d::Identity(), open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm());
//}

int main()
{
    LahgiControl lahgi = LahgiControl::instance();
    lahgi.SetType(eMouduleType::TEST);
    lahgi.LoadListedListModeData("E:\\OneDrive - 한양대학교\\바탕 화면\\CC,CA code\\Cs137_Front_TopLeft_1.5m_60s_LMData.csv");

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

// 프로그램 실행: <Ctrl+F5> 또는 [디버그] > [디버깅하지 않고 시작] 메뉴
// 프로그램 디버그: <F5> 키 또는 [디버그] > [디버깅 시작] 메뉴

// 시작을 위한 팁: 
//   1. [솔루션 탐색기] 창을 사용하여 파일을 추가/관리합니다.
//   2. [팀 탐색기] 창을 사용하여 소스 제어에 연결합니다.
//   3. [출력] 창을 사용하여 빌드 출력 및 기타 메시지를 확인합니다.
//   4. [오류 목록] 창을 사용하여 오류를 봅니다.
//   5. [프로젝트] > [새 항목 추가]로 이동하여 새 코드 파일을 만들거나, [프로젝트] > [기존 항목 추가]로 이동하여 기존 코드 파일을 프로젝트에 추가합니다.
//   6. 나중에 이 프로젝트를 다시 열려면 [파일] > [열기] > [프로젝트]로 이동하고 .sln 파일을 선택합니다.

