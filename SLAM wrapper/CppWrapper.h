#pragma once
#include "LogWrapperCaller.h"
#include "EnergySpectrumData.h"

#include <tuple>

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

		typedef struct BitmapUnmanaged
		{
			uint8_t* ptr;
			int width; 
			int height;
			int step;
			int channelSize;
		}sBitMapUnmanged;


		class LahgiCppWrapper
		{
		private:
			LahgiCppWrapper() {};
		public:
			
			bool SetType(eModuleCppWrapper type);
			
			void SetEchks(std::vector<std::vector<double>>  echks);
			void AddListModeDataWithTransformation(const unsigned short* byteData);

			std::vector< ListModeDataCppWrapper> GetRelativeListModeData();
			void ResetListedListModeData();
			void RestEnergySpectrum(int channelNumber);

			std::tuple<double, double, double> GetEcalValue(int fpgaChannelNumber);
			void SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal);

			size_t GetListedListModeDataSize();

			std::vector<BinningEnergy> GetSpectrum(int channelNumber);
			std::vector<BinningEnergy> GetSumSpectrum();
			std::vector<BinningEnergy> GetAbsorberSumSpectrum();
			std::vector<BinningEnergy> GetScatterSumSpectrum();
			std::vector<BinningEnergy> GetScatterSumSpectrum(int time);
			std::vector<BinningEnergy> GetAbsorberSumSpectrum(int time);

			bool SaveListedListModeData(std::string filePath);
			bool LoadListedListModeData(std::string filePath);
			
			/// <summary>
			/// uint8_t* outImgPtr, int outWidth, int outHeight, int outStep, int outChannelSize
			/// </summary>
			/// <returns>uint8_t* outImgPtr, int outWidth, int outHeight, int outStep, int outChannelSize</returns>
			sBitMapUnmanged GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter);

			/// <summary>
			/// 
			/// </summary>
			/// <returns>Coded Compton Hybrid</returns>
			std::tuple<sBitMapUnmanged, sBitMapUnmanged, sBitMapUnmanged>  GetRadiation2dImage(int timeInMiliSeconds, double s2M, double det_W, double resImprov, double m2D, double hFov, double wFov, int imgSize, double minValuePortion);

			sBitMapUnmanged GetTransPoseRadiationImage(int timeInMiliSeconds, double minValuePortion, double resolution);


			static LahgiCppWrapper& instance();
		};

		class RtabmapCppWrapper
		{
		private:
			RtabmapCppWrapper() {};
			uint8_t* mColorImg = nullptr;
		public:
			
			bool GetIsSlamPipeOn();
			bool GetIsVideoStreamOn();
			

			bool Initiate();

			std::vector<ReconPointCppWrapper> GetRTPointCloud();
			std::vector<ReconPointCppWrapper> GetRTPointCloudTransposed();

			bool GetCurrentVideoFrame(uint8_t** outImgPtr, int* outWidth, int* outHeight, int* outStep, int* outChannelSize);

			void StartVideoStream();
			void StopVideoStream();

			bool StartSlamPipe();
			void StopSlamPipe();
			void ResetSlam();

			bool LoadPlyFile(std::string filePath);
			void SavePlyFile(std::string filePath);
			

			std::vector<ReconPointCppWrapper> GetSlamPointCloud();

			std::vector<ReconPointCppWrapper> GetLoadedPointCloud();

			std::vector<double> getMatrix3DOneLineFromPoseData();

			std::vector<ReconPointCppWrapper> GetReconSLAMPointCloud(double time, eReconCppWrapper reconType, double voxelSize, bool useLoaded);
			std::vector<std::vector<double>> GetOptimizedPoses();
			static RtabmapCppWrapper& instance();
		};


	};
};
