#include "RadiationImage.h"

using namespace Eigen;

// Makting Detector Response Image
constexpr double Det_W = 0.300;
constexpr double Mask_W = 0.370;
constexpr double Mpix = 37;
constexpr double S2M = 1;
constexpr double M2D = 0.05;
constexpr double SP = S2M - M2D;// Source to Mask distance(mm)
constexpr double M = 1 + M2D / S2M; // projection ratio((a + b) / a)
constexpr double Dproj = Det_W / (Mask_W / Mpix * M); // projection mask to Detector pixel Length(mm)
constexpr double reconPlaneWidth = S2M / M2D * Det_W;
constexpr double ResImprov = 5;
int pixelCount = static_cast<int>(round(Dproj * ResImprov));

inline int findIndex(double value, double min, double pixelSize)
{
	if (value - min <= 0)
	{
		return -1;
	}
	return static_cast<int>(floor((value - min) / pixelSize));
}

static cv::Mat CodedMaskMat()
{
	static bool isMaskMade = false;
	static cv::Mat mask;
	if (isMaskMade)
	{
		return mask;
	}
	else
	{
		mask = cv::Mat(37, 37, CV_32S);
		for (int i = 0; i < 37; ++i)
		{
			for (int j = 0; j < 37; ++j)
			{
				if (HUREL::Compton::mCodeMask[i][j])
				{
					mask.at<int>(i, j) = 1;
				}
				else
				{
					mask.at<int>(i, j) = -1;
				}
			}
		}
		isMaskMade = true;
		return mask;
	}
}

void HUREL::Compton::RadiationImage::ShowCV_32SAsJet(cv::Mat img, int size)
{
	if (img.type() != CV_32S)
	{
		return;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::Mat showImg;

	cv::resize(colorImg, showImg, cv::Size(size, size), 0, 0, cv::INTER_NEAREST_EXACT);
	cv::imshow("img", showImg);
	cv::waitKey(0);
}
cv::Mat HUREL::Compton::RadiationImage::GetCV_32SAsJet(cv::Mat img, int size)
{
	cv::Mat showImg;
	if (img.type() != CV_32S)
	{
		return showImg;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<int>(i, j)) - minValue) / (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);


	cv::resize(colorImg, showImg, cv::Size(size, size), 0, 0, cv::INTER_NEAREST_EXACT);
	
	return showImg;

}

HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData> data)
{
	Mat responseImg(pixelCount, pixelCount, CV_32S, Scalar(0));
	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(1));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
	__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));
	int codedImageCount = 0;
	int comptonImageCount = 0;
	for (ListModeData lm : data)
	{
		if (lm.Type == eInterationType::CODED)
		{
			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, -Det_W / 2, Det_W / pixelCount);
			int iY = findIndex(interactionPoseY, -Det_W / 2, Det_W / pixelCount);
			if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
			{
				++responseImgPtr[pixelCount * iY + iX];
				++codedImageCount;
			}
		}
		
		
		if (lm.Type == eInterationType::COMPTON)
		{	
			if (lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 200)
			{
				continue;
			}
			++comptonImageCount;
			for (int i = 0; i < pixelCount; ++i)
			{
				for (int j = 0; j < pixelCount; ++j)
				{
					double imagePlaneX = reconPlaneWidth / pixelCount * i + reconPlaneWidth / pixelCount / 2 * 3- reconPlaneWidth / 2;
					double imagePlaneY = reconPlaneWidth / pixelCount * j + reconPlaneWidth / pixelCount - reconPlaneWidth / 2;
					double imagePlaneZ = S2M + M2D + 0.02;
					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint);
				}
			}

		}
	}
	std::cout << "Lm Count: " << data.size() << " Coded count: " << codedImageCount << " Compton count: " << comptonImageCount << std::endl;
	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * ResImprov, 37 * ResImprov), 0, 0, INTER_NEAREST_EXACT);
	Mat reconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);
	
	reconImg = -reconImg;	
	double maxValue;
	cv::minMaxLoc(reconImg, nullptr, &maxValue);
	Mat idxImg(pixelCount, pixelCount, CV_32S, Scalar(maxValue*0.1));

	cv::max(reconImg, idxImg, mCodedImage);
	
	//mCodedImage = reconImg;

	mDetectorResponseImage = responseImg;
	mComptonImage = comptonImg;

	mHybridImage = mCodedImage.mul(mComptonImage);
	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;

	//ShowCV_32SAsJet(mDetectorResponseImage, 1000);
	//ShowCV_32SAsJet(mCodedImage, 1000);
	//ShowCV_32SAsJet(mComptonImage, 1000);
	//ShowCV_32SAsJet(mHybridImage, 1000);
}

double HUREL::Compton::RadiationImage::OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode)
{
	Eigen::Matrix4d t265toLACCPosTransform;
	t265toLACCPosTransform << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;

	constexpr double imagePlaneZ = S2M + M2D + 0.011;
	Eigen::Vector3d detectorNormalVector(0, 0, 1);
	Eigen::Vector4d point4d(point.x(), point.y(), point.z(), 1);
	Eigen::Vector4d transformedPoint = ((mDetectorTransformation).inverse()* point4d);
	if (transformedPoint.z() <= imagePlaneZ || transformedPoint.z() >= 5)
	{
		//std::cout << transformedPoint << std::endl;
		return 0;
	}
	double xPoseOnImgPlane = transformedPoint.x() * imagePlaneZ / transformedPoint.z();
	double yPoseOnImgPlane = transformedPoint.y() * imagePlaneZ / transformedPoint.z();


	int iX = findIndex(xPoseOnImgPlane, -reconPlaneWidth / 2, reconPlaneWidth / pixelCount);
	int iY = findIndex(yPoseOnImgPlane, -reconPlaneWidth / 2, reconPlaneWidth / pixelCount);


	if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
	{
		__int32 value = 0;
		switch (mode)
		{
		case HUREL::Compton::eRadiationImagingMode::CODED:
			value = static_cast<__int32*>(static_cast<void*>(mCodedImage.ptr()))[pixelCount * (pixelCount - iY) + pixelCount - iX];
			break;
		case HUREL::Compton::eRadiationImagingMode::COMPTON:
			value = static_cast<__int32*>(static_cast<void*>(mComptonImage.ptr()))[pixelCount * (pixelCount - iY) + pixelCount - iX];
		case HUREL::Compton::eRadiationImagingMode::HYBRID:
			value = static_cast<__int32*>(static_cast<void*>(mHybridImage.ptr()))[pixelCount * (pixelCount - iY) + pixelCount - iX];
			break;
		default:
			assert(false);
			return 0.0;
			break;
		}
		return static_cast<double>(value);
	}
	else
	{
		return 0.0;
	}
	



	
}

