#pragma once

#include <queue>
#include <iostream>

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/PoseGraph.h>
#include <open3d/pipelines/registration/Feature.h>
#include <open3d/pipelines/registration/FastGlobalRegistration.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <open3d/pipelines/registration/GlobalOptimization.h>
#define PC_TRANSPOS_TUPLE std::tuple<open3d::geometry::PointCloud, Eigen::Matrix4d>
#define PC_FEATURE_TUPLE std::tuple<open3d::geometry::PointCloud, open3d::pipelines::registration::Feature>
#define PC_FEATURE_TRANSPOS_TUPLE std::tuple<open3d::geometry::PointCloud, open3d::pipelines::registration::Feature, Eigen::Matrix4d>
#define MAX_CORRESPONDENCE_DISTANCE_COARSE 0.10
#define MAX_CORRESPONDENCE_DISTANCE_FINE 0.05
#define VOXEL_SIZE 0.05


class SLAMRobustRecon
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
	std::vector<PC_TRANSPOS_TUPLE> mReconPCs;
	open3d::geometry::PointCloud mSLAMEDPointCloud;
	
	bool mIsSLAMOn = false;
	void RobustReconPipe();
	const static std::tuple<bool, Eigen::Matrix4d_u, Eigen::Matrix6d> RobustReconPairwayRegisteration(open3d::geometry::PointCloud& sourcePC, open3d::geometry::PointCloud& targetPC, open3d::pipelines::registration::Feature& sourceFeature, open3d::pipelines::registration::Feature& targetFeature);
	Eigen::Matrix4d mInitOdementry;

public:
	
	SLAMRobustRecon();
	
	static open3d::geometry::PointCloud MultiwayRegisteration(std::vector<open3d::geometry::PointCloud>& pcs, std::vector<Eigen::Matrix4d>& trs);
	static std::tuple<Eigen::Matrix4d_u, Eigen::Matrix6d> PairwayRegisteration(open3d::geometry::PointCloud& source, open3d::geometry::PointCloud& target, double maxCorrDisCoarse, double maxCorrDisFine, Eigen::Matrix4d initMatirx, bool isPointToPointUsed);
	void StartRobustRecon(Eigen::Matrix4d initOdementry);
	void StopRobustRecon();
	void AddRobustReconPoints(PC_TRANSPOS_TUPLE pcTr);
	open3d::geometry::PointCloud GetSLAMEDPointCloud();
	

};

