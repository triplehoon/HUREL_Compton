#include "TestFuncs.h"

using namespace cv;
using namespace HUREL::Compton;
static Mat CodedMaskMat();
void ShowCV_32SAsJet(Mat img, int size);


void TestFuncs::TestImageRecon()
{
    std::shared_ptr < open3d::geometry::PointCloud> pc = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloudOption opt;

    std::cout << "Loading pointcloud \n";

    if (open3d::io::ReadPointCloudFromPLY("20220706_DigitalLabScan_100uCi_-1,0,2.4_Pointcloud.ply", *pc, opt))
    {
        std::cout << "Show pointcloud \n";

        //open3d::visualization::DrawGeometries({ pc });
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
        RGBA_t rgb = HUREL::Compton::ReconPointCloud::ColorScaleJet(reconPC.reconValues_[i], 0.0 * reconPC.maxReoconValue, reconPC.maxReoconValue);
        pc->colors_[i](0) = rgb.R;
        pc->colors_[i](1) = rgb.G;
        pc->colors_[i](2) = rgb.B;


    }
    open3d::visualization::DrawGeometries({ pc });

    std::cout << "Done \n";

}

void TestFuncs::TestAddingLmData()
{
    LahgiControl& control = HUREL::Compton::LahgiControl::instance();
    control.SetType(eMouduleType::QUAD);
    unsigned short byteData[144];
    for (int j = 0; j < 144; ++j)
    {
        byteData[j] = 0;
    }
    //Channel 4 to 8
    for (int i = 4; i < 5; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            byteData[i * 9 + j] =10;
        }
    }

    //Channel 12 to 16
    for (int i = 12; i < 13; ++i)
    {
        for (int j = 0; j < 9; ++j)
        {
            byteData[i * 9 + j] =10;
        }
    }

    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

    sEnergyCheck echk;
    echk.minE = 0; echk.maxE = 1000000000;
    std::vector<sEnergyCheck> echks;
    echks.push_back(echk);

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
    #pragma omp parallel for
    for (int i = 0; i < 10000000; ++i)
    {
        control.AddListModeDataEigen(byteData, trans, echks);
    }

    std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

    // 초 단위 (소수점으로 표현)
    std::chrono::milliseconds milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    

    printf("%d ms \n", milisec);
}
