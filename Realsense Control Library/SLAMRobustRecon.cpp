#include "SLAMRobustRecon.h"
#include <thread>
#include <future>
#include <fstream>
std::mutex mReconPCsMutex;

using namespace std;
using namespace open3d::pipelines::registration;

SLAMRobustRecon::SLAMRobustRecon()
{
}

open3d::geometry::PointCloud SLAMRobustRecon::MultiwayRegisteration(std::vector<open3d::geometry::PointCloud>& pcs, std::vector<Eigen::Matrix4d>& trs)
{
	
	auto poseGraph = new open3d::pipelines::registration::PoseGraph();

	Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity();

	poseGraph->nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odometry));
	for (int i = 0; i < pcs.size(); ++i) {
		pcs[i] = *std::get<0>(pcs[i].RemoveStatisticalOutliers(20, 5))->VoxelDownSample(VOXEL_SIZE / 2);
		pcs[i].EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(VOXEL_SIZE / 2 * 2, 30));
	}

	for (int i = 0; i < pcs.size(); ++i) {
		auto sourcePc = pcs[i];
		
		for (int j = i + 1; j < pcs.size(); ++j) {
			auto targePc = pcs[j];
			Eigen::Matrix4d initTm = trs[j].inverse() * trs[i];
			auto pairwiseReg = PairwayRegisteration(sourcePc, targePc, MAX_CORRESPONDENCE_DISTANCE_COARSE, MAX_CORRESPONDENCE_DISTANCE_FINE, initTm, true);
			if (j == i + 1) {
				odometry = std::get<0>(pairwiseReg) * odometry;
				poseGraph->nodes_.push_back(
					open3d::pipelines::registration::PoseGraphNode(odometry.inverse())
				);
				poseGraph->edges_.push_back(
					open3d::pipelines::registration::PoseGraphEdge(
						i, j, std::get<0>(pairwiseReg), std::get<1>(pairwiseReg), false)
				);
			}
			else {
				poseGraph->edges_.push_back(
					open3d::pipelines::registration::PoseGraphEdge(
						i, j, std::get<0>(pairwiseReg), std::get<1>(pairwiseReg), true)
				);
			}
		}


	}

	open3d::geometry::PointCloud sumPC;

	GlobalOptimizationConvergenceCriteria criteria;
	GlobalOptimizationOption option(MAX_CORRESPONDENCE_DISTANCE_FINE, 0.25, 0.1);
	GlobalOptimizationLevenbergMarquardt optimization_method;
	GlobalOptimization(*poseGraph, optimization_method, criteria, option);

	for (int i = 0; i < pcs.size(); ++i) {
		sumPC += (pcs[i]).Transform(poseGraph->nodes_[i].pose_);
		/*cout << "MultiwayRegisteration: index = " << i << endl;
		cout << poseGraph->nodes_[i].pose_(0, 0) << ", " << poseGraph->nodes_[i].pose_(0, 1) << ", " << poseGraph->nodes_[i].pose_(0, 2) << ", " << poseGraph->nodes_[i].pose_(0, 3) << endl;
		cout << poseGraph->nodes_[i].pose_(1, 0) << ", " << poseGraph->nodes_[i].pose_(1, 1) << ", " << poseGraph->nodes_[i].pose_(1, 2) << ", " << poseGraph->nodes_[i].pose_(1, 3) << endl;
		cout << poseGraph->nodes_[i].pose_(2, 0) << ", " << poseGraph->nodes_[i].pose_(2, 1) << ", " << poseGraph->nodes_[i].pose_(2, 2) << ", " << poseGraph->nodes_[i].pose_(2, 3) << endl;
		cout << poseGraph->nodes_[i].pose_(3, 0) << ", " << poseGraph->nodes_[i].pose_(3, 1) << ", " << poseGraph->nodes_[i].pose_(3, 2) << ", " << poseGraph->nodes_[i].pose_(3, 3) << endl;
		*/
	}
	
	auto outPC = *std::get<0>(sumPC.RemoveStatisticalOutliers(20, 5));
	outPC = *outPC.VoxelDownSample(VOXEL_SIZE);
	outPC.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(VOXEL_SIZE * 2, 30));
	//outPC->HiddenPointRemoval(Eigen::Vector3d(0, 0, 1), 5);
	delete(poseGraph);
	return outPC;
}

std::tuple<Eigen::Matrix4d_u, Eigen::Matrix6d> SLAMRobustRecon::PairwayRegisteration(open3d::geometry::PointCloud& source, open3d::geometry::PointCloud& target, double maxCorrDisCoarse, double maxCorrDisFine, Eigen::Matrix4d initMatirx, bool isPointToPointUsed)
{
	//Apply point-to-plane ICP
	if (isPointToPointUsed)
	{
		auto icpCoarse = open3d::pipelines::registration::RegistrationICP(source, target, maxCorrDisCoarse, initMatirx, open3d::pipelines::registration::TransformationEstimationPointToPlane());
		auto icpFine = open3d::pipelines::registration::RegistrationColoredICP(source, target, maxCorrDisFine, icpCoarse.transformation_);
		auto informationICP = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target, maxCorrDisFine, icpFine.transformation_);
		return std::make_tuple(icpFine.transformation_, informationICP);
	}
	else 
	{
		auto icpFine = open3d::pipelines::registration::RegistrationColoredICP(source, target, maxCorrDisFine, initMatirx);
		auto informationICP = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target, maxCorrDisFine, icpFine.transformation_);
		return std::make_tuple(icpFine.transformation_, informationICP);
	}	
}

static std::future<void> t1;

void SLAMRobustRecon::StartRobustRecon(Eigen::Matrix4d initOdementry)
{
	mIsSLAMOn = true;
	mInitOdementry = initOdementry;
	auto func = std::bind(&SLAMRobustRecon::RobustReconPipe, this);
	t1 = std::async(std::launch::async, func);

}

void SLAMRobustRecon::StopRobustRecon()
{
	mIsSLAMOn = false;
	t1.get();

	printf("Robust Recon Pipe is end\n");
}

void SLAMRobustRecon::RobustReconPipe()
{
	printf("Start Robust Recon Pipe!1\n");
	int matchedPointsCount = 1;
	auto poseGraph = new open3d::pipelines::registration::PoseGraph();
	ofstream writeFile;
	writeFile.open("Tranformation diff.txt");
	while (mIsSLAMOn)
	{
		mReconPCsMutex.lock();
		std::vector<PC_TRANSPOS_TUPLE> reconPCs = mReconPCs;
		mReconPCsMutex.unlock();

		if (reconPCs.size() == 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			continue;
		}

		open3d::geometry::PointCloud sumPC;

		PoseGraph* poseGraph = new PoseGraph();

		Eigen::Matrix4d_u odometry = get<1>(reconPCs[0]).inverse();

		poseGraph->nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odometry));


		cout << "RobustReconPipe start: " << reconPCs.size() << endl;

		for (int s = 0; s < reconPCs.size(); ++s)
		{			
			//cout << "RobustReconPipe source: " << s << endl;
			auto& sourcePC = get<0>(reconPCs[s]);
			auto& sourceTM = get<1>(reconPCs[s]);
			
			for (int t = s + 1; t < reconPCs.size(); ++t)
			{
				auto& targetPC = get<0>(reconPCs[t]);
				auto& targetTM = get<1>(reconPCs[t]);
				
				if (t == (s + 1)) 
				{
					//odomentry case
					auto result = PairwayRegisteration(sourcePC, targetPC, 0.1 , 0.1,  targetTM.inverse() * sourceTM, false);
					auto& resultTM = get<0>(result);
			
					auto& resultIM = get<1>(result);
					odometry = resultTM * odometry;
					poseGraph->nodes_.push_back(PoseGraphNode(odometry.inverse()));
					poseGraph->edges_.push_back(PoseGraphEdge(s, t, resultTM, resultIM, false));

				}
				else 
				{
					bool isSuccess = true;
					auto& resultTM = targetTM.inverse() * sourceTM;
					auto resultIM = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(sourcePC, targetPC, 0.1, resultTM);

					if (isSuccess)
					{
						poseGraph->edges_.push_back(PoseGraphEdge(s, t, resultTM, resultIM, true));
					}				
				}
			}
		}


		cout << "RobustReconPipe Done: " << reconPCs.size()  << endl;
		GlobalOptimizationConvergenceCriteria criteria;
		GlobalOptimizationOption option(0.3, 0.25, 0.1);
		GlobalOptimizationLevenbergMarquardt optimization_method;
		GlobalOptimization(*poseGraph, optimization_method, criteria, option);
		for (int i = 0; i < reconPCs.size(); ++i) {
			sumPC += open3d::geometry::PointCloud(std::get<0>(reconPCs[i])).Transform(poseGraph->nodes_[i].pose_);
			
		}
		//mReconPCsMutex.lock();
		//for (int i = 0; i < reconPCs.size(); ++i) 
		//{		
		//		mReconPCs[i] = reconPCs[i]; //  std::make_tuple(std::get<0>(reconPCs[i]), poseGraph->nodes_[i].pose_);		//.Transform(poseGraph->nodes_[i].pose_.inverse()
		//}		
		//mReconPCsMutex.unlock();
		matchedPointsCount = reconPCs.size();

		mSLAMEDPointCloud = *(sumPC.VoxelDownSample(VOXEL_SIZE / 5));//.Transform(t265toLACCTransform);
		delete(poseGraph);
	}
	writeFile.close();

	mReconPCsMutex.lock();
	mReconPCs.clear();
	mReconPCsMutex.unlock();

	printf("Stop Robust Recon Pipe!1\n");
}

const std::tuple<bool, Eigen::Matrix4d_u, Eigen::Matrix6d> SLAMRobustRecon::RobustReconPairwayRegisteration(open3d::geometry::PointCloud& sourcePC, open3d::geometry::PointCloud& targetPC, open3d::pipelines::registration::Feature& sourceFeature, open3d::pipelines::registration::Feature& targetFeature)
{
	double MaxCorrespondDistance = 1;/*
	auto option = open3d::pipelines::registration::FastGlobalRegistrationOption(1.4, false, true, MaxCorrespondDistance);
	auto result = open3d::pipelines::registration::FastGlobalRegistration(sourcePC, targetPC, sourceFeature, targetFeature, option);*/
	auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(sourcePC, targetPC, sourceFeature, targetFeature, true, MaxCorrespondDistance);
	if (result.fitness_ < 0.8) {
		std::cout << "No reasonable solution(trace), skip this pair: fitnss_: " << result.fitness_ << std::endl;

		return std::make_tuple(false, Eigen::Matrix4d_u::Identity(), Eigen::Matrix6d::Zero());
	}
	else
	{
		auto information = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(sourcePC, targetPC, MaxCorrespondDistance, result.transformation_);
		if (information(5, 5) / std::fmin(sourcePC.points_.size(), targetPC.points_.size()) < 0.3)
		{
			std::cout << "No reasonable solution(info), skip this pair" << std::endl;
			return std::make_tuple(false, Eigen::Matrix4d_u::Identity(), Eigen::Matrix6d::Zero());
		}
		return std::make_tuple(true, result.transformation_, information);
	}
}

void SLAMRobustRecon::AddRobustReconPoints(PC_TRANSPOS_TUPLE pcTr)
{
	
	//printf("Add robust recon point\n");
	mReconPCsMutex.lock();
	mReconPCs.push_back(pcTr);
	mReconPCsMutex.unlock();
}

open3d::geometry::PointCloud SLAMRobustRecon::GetSLAMEDPointCloud()
{
	std::cout << "Get SLAME PC" << std::endl;
	return mSLAMEDPointCloud;
}




