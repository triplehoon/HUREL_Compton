#include "RadiationImage.h"

using namespace Eigen;

// Makting Detector Response Image
constexpr double Det_W = 0.270;
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
	if (value - min < 0)
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

void ShowCV_32SAsJet(cv::Mat img, int size)
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

HUREL::Compton::RadiationImage::RadiationImage(std::vector<ListModeData> data)
{
	Mat responseImg(pixelCount, pixelCount, CV_32S, Scalar(0));
	Mat comptonImg(pixelCount, pixelCount, CV_32S, Scalar(0));
	__int32* responseImgPtr = static_cast<__int32*>(static_cast<void*>(responseImg.data));
	__int32* comptonImgPtr = static_cast<__int32*>(static_cast<void*>(comptonImg.data));

	for (ListModeData lm : data)
	{
		if (lm.Scatter.InteractionEnergy > 620 && lm.Scatter.InteractionEnergy < 700)
		{
			double& interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
			double& interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];

			int iX = findIndex(interactionPoseX, -Det_W / 2, Det_W / pixelCount);
			int iY = findIndex(interactionPoseY, -Det_W / 2, Det_W / pixelCount);
			if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
			{
				++responseImgPtr[pixelCount * iY + iX];
			}
		}
		
		
		if (lm.Type == eInterationType::COMPTON)
		{
			double e = lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy;
			if (e < 620 || e > 700)
			{
				continue;
			}
			for (int i = 0; i < pixelCount; ++i)
			{
				for (int j = 0; j < pixelCount; ++j)
				{
					double imagePlaneX = reconPlaneWidth / pixelCount * i + reconPlaneWidth / pixelCount / 2 - reconPlaneWidth / 2;
					double imagePlaneY = reconPlaneWidth / pixelCount * j + reconPlaneWidth / pixelCount / 2 - reconPlaneWidth / 2;
					double imagePlaneZ = S2M + M2D + 0.01;
					Eigen::Vector3d imgPoint;
					imgPoint[0] = imagePlaneX;
					imgPoint[1] = imagePlaneY;
					imgPoint[2] = imagePlaneZ;
					comptonImgPtr[pixelCount * (pixelCount - j - 1) + pixelCount - i - 1] += ReconPointCloud::SimpleComptonBackprojectionUntransformed(lm, imgPoint);
				}
			}

		}
	}

	Mat scaleG;
	cv::resize(CodedMaskMat(), scaleG, Size(37 * ResImprov, 37 * ResImprov), 0, 0, INTER_NEAREST_EXACT);
	Mat reconImg;
	cv::filter2D(responseImg, reconImg, CV_32S, scaleG);
	
	reconImg = -reconImg;	
	double maxValue;
	cv::minMaxLoc(reconImg, nullptr, &maxValue);
	Mat idxImg(pixelCount, pixelCount, CV_32S, Scalar(maxValue*0.2));

	cv::max(reconImg, idxImg, mCodedImage);
	


	mDetectorResponseImage = responseImg;
	mComptonImage = comptonImg;

	mHybridImage = mCodedImage.mul(mComptonImage);
	mDetectorTransformation = data[0].DetectorTransformation;
	mListedListModeData = data;

	ShowCV_32SAsJet(mDetectorResponseImage, 1000);
	ShowCV_32SAsJet(mCodedImage, 1000);
	ShowCV_32SAsJet(mComptonImage, 1000);
	ShowCV_32SAsJet(mHybridImage, 1000);
}
