#include "ComptonBPCpp.h"

using namespace HUREL;


bool CComptonBP::CheckEffectedBP(Vector3d scatterPhotonPosition, double scatterPhotonEnergy, Vector3d absorberPhotonPosition, double absorberPhotonEnergy, Vector3d imageSpacePosition, double angleThreshold = m_angleThreshold)
{
    double comptonCal = 1 - 511 * scatterPhotonEnergy / absorberPhotonEnergy / (scatterPhotonEnergy + absorberPhotonEnergy);
    if (comptonCal >= 1 || comptonCal <= -1)
        return false;

    double comptonScatteringAngle = acos(comptonCal) / M_PI * 180;


    Vector3d effectToScatterVector = (scatterPhotonPosition - imageSpacePosition).normalized();
    Vector3d scatterToAbsorberVector = (absorberPhotonPosition - scatterPhotonPosition).normalized();

    double positionDotPord = (effectToScatterVector.adjoint() * (scatterToAbsorberVector)).value();

    double effectedAngle = acos(positionDotPord) / M_PI * 180;

    if (abs(effectedAngle - comptonScatteringAngle) < angleThreshold)
        return true;
    else
        return false;
}


open3d::geometry::PointCloud CComptonBP::BPtoPointCloud(vector<ListModeData>* lmdata, open3d::geometry::PointCloud* imgSpace)
{
    for (int i = 0; i < lmdata->size(); i++)
    {
        for (int j = 0; j < imgSpace->points_.size(); j++)
        {
            Vector3d imageSpacePosition = imgSpace->points_[j];
            auto scatterPhotonPosition = (*lmdata)[i].scatterPhotonPosition;
            auto scatterPhotonEnergy = (*lmdata)[i].scatterPhotonEnergy;
            auto absorberPhotonPosition = (*lmdata)[i].absorberPhotonPosition;
            auto absorberPhotonEnergy = (*lmdata)[i].absorberPhotonEnergy;
            if (CheckEffectedBP(scatterPhotonPosition, scatterPhotonEnergy, absorberPhotonPosition, absorberPhotonEnergy, imageSpacePosition))
                imgSpace->colors_[j] = imgSpace->colors_[j] + Vector3d(0, -1 / 255, -1 / 255);
        }
    }

    return open3d::geometry::PointCloud();
}

