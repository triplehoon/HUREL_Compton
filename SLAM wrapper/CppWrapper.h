#pragma once

#include "EnergySpectrum.h"
#include "LogWrapperCaller.h"

namespace HUREL {
	namespace Compton {
		enum class eModuleCppWrapper
		{
			MONO,
			QUAD,
			QUAD_DUAL
		};
		enum class eReconCppWrapper
		{
			CODED,
			COMPTON,
			HYBRID,
		};

		struct ListModeDataCppWrapper 
		{
			double ScatterRelativeInteractionPointX;
			double ScatterRelativeInteractionPointY;
			double ScatterRelativeInteractionPointZ;
			double ScatterInteractionEnergy;
			double AbsorberRelativeInteractionPointX;
			double AbsorberRelativeInteractionPointY;
			double AbsorberRelativeInteractionPointZ;
			double AbsorberInteractionEnergy;
		};

		

		struct ReconPointCppWrapper
		{
			double pointX;
			double pointY;
			double pointZ;

			double colorR;
			double colorG;
			double colorB;
			double colorA;

			double reconValue;
		};

		class LahgiCppWrapper
		{
		private:
			LahgiCppWrapper() {};
			
		public:
			
			bool SetType(eModuleCppWrapper type);
			void AddListModeDataWithTransformation(const unsigned short* byteData, std::vector<std::vector<double>> echks);
			std::vector< ListModeDataCppWrapper> GetRelativeListModeData();
			void RestListedListModeData();
			void RestEnergySpectrum(int channelNumber);

			std::vector<BinningEnergy> GetSpectrum(int channelNumber);
			std::vector<BinningEnergy> GetSumSpectrum();
			std::vector<BinningEnergy> GetAbsorberSumSpectrum();
			std::vector<BinningEnergy> GetScatterSumSpectrum();
			std::vector<BinningEnergy> GetScatterSumSpectrum(int time);
			std::vector<BinningEnergy> GetAbsorberSumSpectrum(int time);

			bool SaveListedListModeData(std::string filePath);
			bool LoadListedListModeData(std::string filePath);


			static LahgiCppWrapper& instance();
		};

		class RtabmapCppWrapper
		{
		private:
			RtabmapCppWrapper() {};
			
		public:
			
			bool GetIsSlamPipeOn();
			bool GetIsVideoStreamOn();
			

			bool Initiate();

			std::vector<ReconPointCppWrapper> GetRTPointCloud();
			std::vector<ReconPointCppWrapper> GetRTPointCloudTransposed();

			bool GetCurrentVideoFrame(uint8_t* outImgPtr, int* outWidth, int* outHeight, int* outStep, int* outChannelSize);

			void StartVideoStream();
			void StopVideoStream();

			bool StartSlamPipe();
			void StopSlamPipe();
			void ResetSlam();

			bool LoadPlyFile(std::string filePath);

			std::vector<ReconPointCppWrapper> GetSlamPointCloud();

			std::vector<ReconPointCppWrapper> GetLoadedPointCloud();

			std::vector<double> getMatrix3DOneLineFromPoseData();

			std::vector<ReconPointCppWrapper> GetReconSLAMPointCloud(double time, eReconCppWrapper reconType, double voxelSize, bool useLoaded);

			static RtabmapCppWrapper& instance();
		};


	};
};
