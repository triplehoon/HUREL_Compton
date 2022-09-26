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
	int n;
	n = Eigen::nbThreads();
	std::cout << n << "\n";

	LahgiControl& control = HUREL::Compton::LahgiControl::instance();
	control.SetType(eMouduleType::QUAD);

	TestAddingLmDataVerification(10000);

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
			byteData[i * 9 + j] = 10;
		}
	}

	//Channel 12 to 16
	for (int i = 12; i < 13; ++i)
	{
		for (int j = 0; j < 9; ++j)
		{
			byteData[i * 9 + j] = 10;
		}
	}

	Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

	sEnergyCheck echk;
	echk.minE = 0; echk.maxE = 1000000000;
	std::vector<sEnergyCheck> echks;
	echks.push_back(echk);

	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	#pragma omp parallel for
	for (int i = 0; i < 100; ++i)
	{
		control.AddListModeDataWithTransformationVerification(byteData, echks);
		//control.AddListModeDataWithTransformation(byteData, echks);
		//control.AddListModeDataEigen(byteData, Eigen::Matrix4d::Identity(), echks);
	}

	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

	// 초 단위 (소수점으로 표현)
	std::chrono::milliseconds milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	auto listmodeDataFirst = control.GetListedListModeData();
	printf("%d ms \n", milisec);

	start = std::chrono::system_clock::now();
	for (int i = 0; i < 1000000; ++i)
	{
		//control.AddListModeDataWithTransformationVerification(byteData, echks);
		//control.AddListModeDataWithTransformation(byteData, echks);
		control.AddListModeDataWithTransformation(byteData, echks);
		//control.AddListModeDataEigen(byteData, Eigen::Matrix4d::Identity(), echks);
	}

	end = std::chrono::system_clock::now();

	// 초 단위 (소수점으로 표현)
	milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	//

	//Eigen::Matrix<float, Eigen::Dynamic, 9> mXYLogMueIndex;
	//Eigen::Matrix<float, Eigen::Dynamic, 1> mXYSumMueIndex;
	//Eigen::Matrix<float, 9, 1> mGainEigenVector;
	//Eigen::Matrix<float, 9, 1> pmtADCValue;

	//mXYLogMueIndex.resize(145 * 145, 9);
	//mXYSumMueIndex.resize(145 * 145, 1);
	//mXYLogMueIndex.setRandom();
	//mXYLogMueIndex.setRandom();
	//mGainEigenVector.setRandom();

	//start = std::chrono::system_clock::now();

	//for (int i = 0; i < 100000; ++i)
	//{
	//	int maxIdx;
	//	Eigen::Matrix<float, Eigen::Dynamic, 1> mXYSumMueIndexCalc;
	//	//mXYSumMueIndexCalc.resize(mLutSize * mLutSize);
	//	const Eigen::Vector<float, 9> normalizedPMTValue = mGainEigenVector.cwiseProduct(pmtADCValue);

	//	mXYSumMueIndexCalc = mXYLogMueIndex * normalizedPMTValue + mXYSumMueIndex;

	//	maxIdx = mXYSumMueIndexCalc.maxCoeff<Eigen::PropagateNumbers>(&maxIdx);
	//}
	////mXYSumMueIndexCalc.resize(5);
	//end = std::chrono::system_clock::now();
	//milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	//printf("%d ms \n", milisec);
	//
	//Eigen::Matrix<float, 9, 1> mGainEigenVectors;
	//Eigen::Matrix<float, 9, Eigen::Dynamic> pmtADCValues;


	//mXYLogMueIndex.resize(145 * 145, 9);
	//mXYSumMueIndex.resize(145 * 145, 1);
	//mXYLogMueIndex.setRandom();
	//mXYLogMueIndex.setRandom();

	//int blockSize = 100000;

	//pmtADCValues.resize(9, blockSize);
	//pmtADCValues.setRandom();
	//start = std::chrono::system_clock::now();

	//for (int i = 0; i < 1; ++i)
	//{
	//	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> poses;


	//	int maxIdx;
	//	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mXYSumMueIndexCalc;
	//	//mXYSumMueIndexCalc.resize(mLutSize * mLutSize);
	//	mXYSumMueIndexCalc.noalias() = mXYLogMueIndex * pmtADCValues + mXYSumMueIndex;
	//	

	//}
	////mXYSumMueIndexCalc.resize(5);
	//end = std::chrono::system_clock::now();
	//milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	printf("%d ms \n", milisec);
}



void TestFuncs::TestAddingLmDataVerification(int counts)
{
	LahgiControl& control = HUREL::Compton::LahgiControl::instance();
	//control.SetType(eMouduleType::QUAD);
	unsigned short byteData[144];


	for (int j = 0; j < 144; ++j)
	{
		byteData[j] = 0;
	}
	//Channel 4 to 8
	srand(0);
	int failCount = 0;
	double errorDistSum = 0;
	for (int iter = 0; iter < counts; ++iter)
	{


		for (int i = 4; i < 5; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				byteData[i * 9 + j] = (unsigned char)rand();
			}
		}

		//Channel 12 to 16
		for (int i = 12; i < 13; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				byteData[i * 9 + j] = (unsigned char)rand();
			}
		}

		Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

		sEnergyCheck echk;
		echk.minE = 0; echk.maxE = 1000000000;
		std::vector<sEnergyCheck> echks;
		echks.push_back(echk);

		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		//#pragma omp parallel for
		for (int i = 0; i < 1; ++i)
		{
			control.AddListModeDataWithTransformation(byteData, echks);
			//control.AddListModeDataEigen(byteData, Eigen::Matrix4d::Identity(), echks);
		}

		std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

		// 초 단위 (소수점으로 표현)
		std::chrono::milliseconds milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		auto listmodeDataFirst = control.GetListedListModeData();

		control.ResetListedListModeData();

		for (int i = 0; i < 1; ++i)
		{
			control.AddListModeDataWithTransformationVerification(byteData, echks);
		}

		auto listmodeDataSecond = control.GetListedListModeData();

		if (listmodeDataFirst[0].Scatter.RelativeInteractionPoint[0] - listmodeDataSecond[0].Scatter.RelativeInteractionPoint[0] < 0.002 &&
			listmodeDataFirst[0].Scatter.RelativeInteractionPoint[1] - listmodeDataSecond[0].Scatter.RelativeInteractionPoint[1] < 0.002 )
		{
		}
		else
		{
			++failCount;
			errorDistSum += sqrt(
				pow((listmodeDataFirst[0].Scatter.RelativeInteractionPoint[0] - listmodeDataSecond[0].Scatter.RelativeInteractionPoint[0]),2)
				+ pow((listmodeDataFirst[0].Scatter.RelativeInteractionPoint[1] - listmodeDataSecond[0].Scatter.RelativeInteractionPoint[1]),2));
		/*	std::cout << listmodeDataFirst[0].Scatter.RelativeInteractionPoint *1000 << std::endl 
				      << listmodeDataSecond[0].Scatter.RelativeInteractionPoint * 1000 << std::endl << std::endl;
					  */
		}

		control.ResetListedListModeData();

		
	}


	if (failCount == 0)
	{
		std::cout << "All success" << std::endl;
	}
	else 
	{
		std::cout << "Fail rate is " << (double)failCount / counts * 100 << " %" << std::endl;
		std::cout << "error distance is " << errorDistSum / failCount * 1000 << " mm" << std::endl;
	}
	
}
