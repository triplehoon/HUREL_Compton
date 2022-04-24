#pragma once
#include <opencv2/core.hpp>

namespace HUREL {
	namespace Compton {
		//false = block, true = not block 
		//Lahgi axis coordiante	
		const static bool mCodeMask[37][37] = {
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		};
		
		class CodeMaskCalc
		{
		public:
			CodeMaskCalc() {};
			CodeMaskCalc(int pixelCount, double activeAreaSize);

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

		};
	}
}
