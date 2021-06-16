#include "ReconPointCloud.h"



HUREL::Compton::ReconPointCloud::ReconPointCloud(open3d::geometry::PointCloud& pc) :
	open3d::geometry::PointCloud(pc)
{
	size_t size = pc.points_.size();
	reconValues_ = std::vector<double>(size);

}

void HUREL::Compton::ReconPointCloud::CalculateReconPoint(ListModeData lmData, double(*calcFunc)(ListModeData, Eigen::Vector3d))
{

	size_t size = points_.size();
	double maxReconValv = -DBL_MAX;
	for (size_t i = 0; i < size; ++i)
	{
		reconValues_[i] += calcFunc(lmData, points_[i]);
		if (reconValues_[i] > maxReconValv)
		{
			maxReconValv = reconValues_[i];
		}
	}
}

double HUREL::Compton::ReconPointCloud::SimpleBackprojection(ListModeData lmData, Eigen::Vector3d imgPoint)
{
	if (lmData.Type != eInterationType::COMPTON)
	{
		return 0;
	}
	double comptonCal = 1 - 511 * lmData.Scatter.InteractionEnergy / lmData.Absorber.InteractionEnergy / (lmData.Scatter.InteractionEnergy + lmData.Absorber.InteractionEnergy);
	if (comptonCal >= 1 || comptonCal <= -1)
	{ 
		return 0;
	}
	double comptonScatteringAngle = acos(comptonCal) / EIGEN_PI * 180;
	Eigen::Vector3d effectToScatterVector = (imgPoint - lmData.Scatter.TransformedInteractionPoint.head<3>());
	Eigen::Vector3d scatterToAbsorberVector = (lmData.Scatter.TransformedInteractionPoint.head<3>() - lmData.Absorber.TransformedInteractionPoint.head<3>());
	effectToScatterVector.normalize();
	scatterToAbsorberVector.normalize();
	double positionDotPord = effectToScatterVector.dot(scatterToAbsorberVector);

	double effectedAngle = acos(positionDotPord) / EIGEN_PI * 180;

	if (abs(effectedAngle - comptonScatteringAngle) < 5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

HUREL::Compton::RGBA_t HUREL::Compton::ReconPointCloud::ColorScaleJet(double v, double vmin, double vmax)
{
	double dv;

	if (v < vmin)
	{
		v = vmin;
		RGBA_t rgba = {0, 0, 0, 0};
		return rgba;
	}
	if (v > vmax)
	{
		v = vmax;
	}
	dv = vmax - vmin;
	if (dv == 0)
	{
		RGBA_t rgba = { 0, 0, 0, 0 };
		return rgba;
	}
	double r = 1.0f, g = 1.0f, b = 1.0f;
	if (v < (vmin + 0.25 * dv))
	{
		r = 0;
		g = 4 * (v - vmin) / dv;
	}
	else if (v < (vmin + 0.5 * dv))
	{
		r = 0;
		b = 1 + 4 * (vmin + 0.25f * dv - v) / dv;
	}
	else if (v < (vmin + 0.75 * dv))
	{
		r = 4 * (v - vmin - 0.5f * dv) / dv;
		b = 0;
	}
	else
	{
		g = 1 + 4 * (vmin + 0.75f * dv - v) / dv;
		b = 0;
	}
	double a = 0.5;
	RGBA_t rgba_out = {r, g, b, a};
	return rgba_out;
}
