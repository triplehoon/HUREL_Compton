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
	echk.minE = 0; echk.maxE = 10000000000000;
	std::vector<sEnergyCheck> echks;
	echks.push_back(echk);
	control.SetEchk(echks);

	control.ResetListedListModeData();
	for (int i = 0; i < 16; ++i)
	{
		control.ResetEnergySpectrum(0);
	}
	std::array<unsigned short, 144> pushData;


	memcpy(&pushData.front(), byteData, 144 * sizeof(unsigned short));
	std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
	
	#pragma omp parallel for
	for (int i = 0; i < 10000000; ++i)
	{
		control.AddListModeDataWithTransformationLoop(pushData);
	}

	std::chrono::system_clock::time_point end = std::chrono::system_clock::now();

	// 초 단위 (소수점으로 표현)
	std::chrono::milliseconds milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

	auto listmodeDataFirst = control.GetListedListModeData();
	printf("%d ms [%f kHz, %d] \n", milisec.count(), (float)control.GetListedListModeData().size() / milisec.count(), control.GetListedListModeData().size());
	control.ResetListedListModeData();
	for (int i = 0; i < 16; ++i)
	{
		control.ResetEnergySpectrum(0);
	}
	start = std::chrono::system_clock::now();
	
	for (int i = 0; i < 10000000; ++i)
	{
		control.AddListModeDataWithTransformation(byteData);
	}
	while (control.GetListedListModeDataSize() < 10000000 * 0.99)
	{

	}
	end = std::chrono::system_clock::now();

	// 초 단위 (소수점으로 표현)
	milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


	printf("%d ms [%f kHz, %d] \n", milisec.count(), (float)control.GetListedListModeData().size() / milisec.count(), control.GetListedListModeData().size());
}


inline int findIndex(double value, double min, double pixelSize)
{
	if (value - min <= 0)
	{
		return -1;
	}
	return static_cast<int>(floor((value - min) / pixelSize));
}

void TestFuncs::GetFloodImage()
{
	LahgiControl& control = HUREL::Compton::LahgiControl::instance();
	control.SetType(eMouduleType::QUAD);


	srand(0);
	unsigned short byteData[144];
	for (int j = 0; j < 144; ++j)
	{
		byteData[j] = 0;
	}
	

	Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

	sEnergyCheck echk;
	echk.minE = 0; echk.maxE = 10000000000000;
	std::vector<sEnergyCheck> echks;
	echks.push_back(echk);
	control.SetEchk(echks);
	control.ResetListedListModeData();
	for (int i = 0; i < 16; ++i)
	{

	
		control.ResetEnergySpectrum(0);
	}
	std::chrono::system_clock::time_point  start = std::chrono::system_clock::now();

	for (int iter = 0; iter < 1000000; ++iter)
	{
		for (int j = 0; j < 144; ++j)
		{
			byteData[j] = 0;
		}

		int index = floor((double)rand() / RAND_MAX * 4);

		//Channel 4 to 8
		for (int i = 4 + index; i < 5 + index; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				byteData[i * 9 + j] = (unsigned char)rand();
			}
		}
		index = floor((double)rand() / RAND_MAX * 4);
		//Channel 12 to 16
		for (int i = 12 + index; i < 13 + index; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				byteData[i * 9 + j] = (unsigned char)rand();
			}
		}
		control.AddListModeDataWithTransformation(byteData);
	}
	while (control.GetListedListModeDataSize() < 1000000*0.99)
	{

	}
	std::chrono::system_clock::time_point  end = std::chrono::system_clock::now();

	// 초 단위 (소수점으로 표현)
	std::chrono::milliseconds milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);


	printf("%d ms [%f kHz, %d] \n", milisec.count(), (float)control.GetListedListModeData().size() / milisec.count(), control.GetListedListModeData().size());

	cv::Mat img = control.GetResponseImage(1000, 100);

	cv::imshow("img", img);
	cv::waitKey(0);
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
		control.SetEchk(echks);

		std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
		//#pragma omp parallel for
		for (int i = 0; i < 1; ++i)
		{
			control.AddListModeDataWithTransformation(byteData);
			//control.AddListModeDataEigen(byteData, Eigen::Matrix4d::Identity(), echks);
		}
		while (control.GetListedListModeData().size() != 1)
		{

		}

		std::chrono::system_clock::time_point end = std::chrono::system_clock::now();


		
		auto listmodeDataFirst = control.GetListedListModeData();

		// 초 단위 (소수점으로 표현)
		std::chrono::milliseconds milisec = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		

		control.ResetListedListModeData();

		for (int i = 0; i < 1; ++i)
		{
			control.AddListModeDataWithTransformationVerification(byteData);
		}
		while (control.GetListedListModeData().size() != 1)
		{

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

struct InteractionDataTest
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector4f RelativeInteractionPoint = Eigen::Vector4f(nan(""), nan(""), nan(""), 1);
	Eigen::Vector4f TransformedInteractionPoint = Eigen::Vector4f(nan(""), nan(""), nan(""), 1);
	float InteractionEnergy = 0;
};


class ListModeDataTest
{
private:

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		eInterationType Type = eInterationType::NONE;
	InteractionDataTest Scatter;
	InteractionDataTest Absorber;
	//time_t InteractionTime = 0;
	std::chrono::milliseconds InteractionTimeInMili = std::chrono::milliseconds(0);
	Eigen::Matrix4f DetectorTransformation = Eigen::Matrix4f::Zero();

};



void TestFuncs::TestZlib()
{
	ListModeData i[1000];
	for (int iter = 0; iter < 1000; ++iter)
	{
	}
	uchar* dataBuffer = new uchar[sizeof(i) * 1.01 + 12];
	unsigned long dataBufferSize = sizeof(i) * 1.01 + 12;
	int test = compress(dataBuffer, &dataBufferSize, (uchar*)(& i), sizeof(i));

	switch (test)
	{
	case Z_OK:
		printf("%lu to %lu \n", sizeof(i), dataBufferSize);
		break;
	case Z_MEM_ERROR:
		printf("Memory Error \n");
		break;
	case Z_BUF_ERROR:
		printf("Buff Error \n");
		break;
	}
	uchar* dataCompress = new uchar[dataBufferSize];
	memcpy(dataCompress, dataBuffer, dataBufferSize);
	delete dataBuffer;

	uchar* dataBufferUncompress = new uchar[sizeof(i)];
	
	unsigned long dataBufferUncompressSize = sizeof(i);
	test = uncompress(dataBufferUncompress, &dataBufferUncompressSize, dataCompress, dataBufferSize);
	delete dataCompress;

	switch (test)
	{
	case Z_OK:
		printf("%lu to %lu \n", dataBufferSize, dataBufferUncompressSize);
		break;
	case Z_MEM_ERROR:
		printf("Memory Error \n");
		break;
	case Z_BUF_ERROR:
		printf("Buff Error \n");
		break;
	}
	for (int iter = 0; iter < 1000; ++iter)
	{
		i[iter] = *(ListModeData*)(dataBufferUncompress + iter*sizeof(ListModeData));
		std::cout << i[iter].WriteListModeData() << std::endl;
	}

	delete dataBufferUncompress;
	
}


void TestFuncs::LoadSystemMatrix()
{
	MatrixXd systemMatrix;
	std::string filePath = "E:\\OneDrive - 한양대학교\\01.Hurel\\01.현재작업\\20211209 쿼드 시뮬레이션 논문 작성\\codes\\SimulationMlem\\data\\AngleInteractionProbData(NaIScatter,130,1,300,5,dist,3m).bin";
	FILE* pFile = fopen(filePath.c_str() , "rb"); //read mode 
	if (pFile == NULL)
	{
		printf("Nofile");
		return;
	}

	////10, 30, 50, 100, 200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000 keV 

	int energyIndex = 6;

	double buffer[23];
	systemMatrix = MatrixXd(17161, 3600);
	//						detetor pos(i, j)
	//			  (-0.16, -0.16) (-0.12, -0.16) (-0.08, -0.16) ......
	// (azm, alt) --------------------------------------------------
	// (-65, -65) |
	// (-60, -65) | System Matrix
	// (-55, -65) |
	
	int indexIazm = 0;
	for (double azm = -65; azm <= 65; ++azm)
	{
		int indexIalt = 0;
		for (double alt = -65; alt <= 65; ++alt)
		{
			int indexIx = 0;
			for (double x = -147.5; x <= 147.5; x += 5)
			{
				int indexIy = 0;
				for (double y = -147.5; y <= 147.5; y += 5)
				{
					int test = fread(buffer, sizeof(double), 23, pFile);
					double value =  buffer[energyIndex + 3];
					int i = indexIazm + indexIalt * 131;
					int j = indexIx + indexIy * 60;
					systemMatrix(i, j) = value;
					++indexIy;
				}
				++indexIx;
			}
			++indexIalt;
		}
		++indexIazm;
	}
	
	MatrixXd sensitivityMatrix = systemMatrix.rowwise().sum();

	

	fclose(pFile);          //파일 닫기
	return;

}