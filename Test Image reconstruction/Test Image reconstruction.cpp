// Test Image reconstruction.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#include "RealsenseControl.h"
#include "LahgiControl.h"
#include "CodeMaskCalc.h"
#include "RadiationImage.h"



using namespace cv;
using namespace HUREL::Compton;
static Mat CodedMaskMat();
Mat MakeDetectorResponse(std::vector<ListModeData> lmData);
void ShowCV_32SAsJet(Mat img, int size);

#include <stdio.h>
#include <open3d/visualization/utility/DrawGeometry.h>

int main()
{
    
    std::shared_ptr < open3d::geometry::PointCloud> pc = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloudOption opt;

    std::cout << "Loading pointcloud \n";

    if (open3d::io::ReadPointCloudFromPLY("20220706_DigitalLabScan_100uCi_-1,0,2.4_Pointcloud.ply", *pc, opt))
    {
        std::cout << "Show pointcloud \n";
        
        open3d::visualization::DrawGeometries({ pc });
    }
    else
    {
        std::cout << "Fail to load point cloud\n";
    }

    Vector3d test;
    test(0) = 1;
    test(1) = 2;
    test(2) = 3;
    test.normalize();
     std::cout << "Load LAHGI \n";
    LahgiControl& lahgi = LahgiControl::instance();
    lahgi.SetType(eMouduleType::TEST);

    std::cout << "Load listmode data \n";
    lahgi.LoadListedListModeData("20220706_DigitalLabScan_100uCi_-1,0,2.4_cpplmdata.csv");
    
    std::vector<ListModeData> lmData = lahgi.GetListedListModeData();
    std::cout << "Done loading listmode data: " << lmData.size() << std::endl;
   
    std::vector<RadiationImage> radimgs;

    std::chrono::milliseconds startTime = lmData[0].InteractionTimeInMili;
    int startIdx = 0;
    HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(*pc);

    #pragma omp parallel for
    for (int i = 0; i < lmData.size(); ++i)
    {
    /*   if (lmData[i].InteractionTimeInMili != startTime)
        {

            std::vector<ListModeData>::const_iterator first = lmData.begin() + startIdx;
            std::vector<ListModeData>::const_iterator last = lmData.begin() + i ;
            std::vector<ListModeData> sameTimeData(first, last);
            std::vector<ListModeData> effectiveData;
            for (const auto lm : sameTimeData)
            {
                if (lm.Type == eInterationType::CODED)
                {
                    if (lm.Scatter.InteractionEnergy > 600 && lm.Scatter.InteractionEnergy < 720)
                    {
                        effectiveData.push_back(lm);
                    }
                }
                else if (lm.Type == eInterationType::COMPTON)
                {
                    if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy > 600 && lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 720)
                    {
                        effectiveData.push_back(lm);
                    }
                }
            }
            if (effectiveData.size() == 0)
            {
                startTime = lmData[i].InteractionTimeInMili;
                startIdx = i;
                continue;
            }
           

            radimgs.push_back(img);

            startTime = lmData[i].InteractionTimeInMili;
            startIdx = i;
        }*/
        if (lmData[i].Type == eInterationType::COMPTON)
        {
            if (lmData[i].Scatter.InteractionEnergy + lmData[i].Absorber.InteractionEnergy > 600 && lmData[i].Scatter.InteractionEnergy + lmData[i].Absorber.InteractionEnergy < 720)
            {
                reconPC.CalculateReconPoint(lmData[i], HUREL::Compton::ReconPointCloud::SimpleComptonBackprojection);
            }
        }
    }


    std::shared_ptr < open3d::geometry::PointCloud> reconpc_ptr = std::make_shared<open3d::geometry::PointCloud>();
    
    for (int i = 0; i < reconPC.colors_.size(); ++i)
    {
        //Eigen::Matrix<double, 3, 1, Eigen::DontAlign> color;
        //Eigen::Matrix<double, 3, 1, Eigen::DontAlign> point;
        Vector3d color;
        Vector3d point;
        RGBA_t rgb = HUREL::Compton::ReconPointCloud::ColorScaleJet(reconPC.reconValues_[i], 0, reconPC.maxReoconValue);
        pc->colors_[i](0) = rgb.R;
        pc->colors_[i](1) = rgb.G;
        pc->colors_[i](2) = rgb.B;
        
     
    }
    open3d::visualization::DrawGeometries({ pc });
    open3d::io::WritePointCloudOption opt2;
    open3d::io::WritePointCloudToPLY("comptonImg.ply", *pc, opt2);



    std::cout << "Done \n";
    return 0;
}