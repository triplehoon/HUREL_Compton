// TestDataAqusition.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//

#include <iostream>
#include "Module.h"
#include <chrono>


using namespace HUREL::Compton;


int main()
{


    Eigen::Matrix4d testTransform;
    Eigen::Vector4d point3d;
    point3d << 1, 1, 1;


    Eigen::Vector4d point4d = Eigen::Vector4d(1,1,1,1);



    testTransform << -1, 1, 3, 1,
        0.5, 1, 1, 1,
        0.2, 2, -1, 1,
        0, 0, 0, 1;  
    
    // 측정 시작 위치
   std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
   Eigen::Vector4d transformedPoint;

   for (long long int i = 0; i < 100000000000; ++i)
   {
      transformedPoint = testTransform * point4d + transformedPoint;
   }

   // 측정 종료 위치
   std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;

    std::cout << "Time takes " << sec.count() << " SEC\n";
    std::cout << transformedPoint[0] << ", " << transformedPoint[1] << ", " << transformedPoint[2] << ", " << transformedPoint[3] << std::endl;

        //double gain[9] {1, 1, 1, 1, 1, 1, 1, 1, 1};
    //Module* module = new Module(eMouduleType::QUAD, gain, gain, "MonoScatterLUT.csv", 0, 0, 0, 5, 3000);
    //Module& moduleE = *module;

    //// 측정 시작 위치
    //std::chrono::system_clock::time_point start = std::chrono::system_clock::now();


    //unsigned short pmt[9]{ 200, 200, 200, 200, 200, 200, 200, 200, 200 };

    //for (int i = 0; i < 100000; ++i)
    //{
    //    auto ans = moduleE.FastMLPosEstimation(pmt);
    //}

    //// 측정 종료 위치
    //std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;

    //

    //std::cout << "Time takes " << sec.count() << " SEC\n";

    //while (true)
    //{

    //}
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
