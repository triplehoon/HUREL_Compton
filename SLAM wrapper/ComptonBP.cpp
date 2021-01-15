#include "ComptonBP.h"


using namespace HUREL;


bool CComptonBP::CheckEffectedBP(Eigen::Vector3d scatterPhotonPosition, double scatterPhotonEnergy, 
    Eigen::Vector3d absorberPhotonPosition, double absorberPhotonEnergy, 
    Eigen::Vector3d imageSpacePosition, double angleThreshold)
{
    double comptonCal = 1 - 511 * scatterPhotonEnergy / absorberPhotonEnergy / (scatterPhotonEnergy + absorberPhotonEnergy);
    if (comptonCal >= 1 || comptonCal <= -1)
        return false;

    double comptonScatteringAngle = acos(comptonCal) / M_PI * 180;


    Eigen::Vector3d effectToScatterVector = (scatterPhotonPosition - imageSpacePosition).normalized();
    Eigen::Vector3d scatterToAbsorberVector = (absorberPhotonPosition - scatterPhotonPosition).normalized();

    double positionDotPord = (effectToScatterVector.adjoint() * (scatterToAbsorberVector)).value();

    double effectedAngle = acos(positionDotPord) / M_PI * 180;

    if (abs(effectedAngle - comptonScatteringAngle) < angleThreshold)
        return true;
    else
        return false;
}


open3d::geometry::PointCloud CComptonBP::BPtoPointCloud(std::vector<ListModeData>* lmdata, open3d::geometry::PointCloud* imgSpace, double angleThreshold =5)
{
    for (int i = 0; i < lmdata->size(); i++)
    {
        for (int j = 0; j < imgSpace->points_.size(); j++)
        {
            Eigen::Vector3d imageSpacePosition = imgSpace->points_[j];
            auto scatterPhotonPosition = (*lmdata)[i].scatterPhotonPosition;
            auto scatterPhotonEnergy = (*lmdata)[i].scatterPhotonEnergy;
            auto absorberPhotonPosition = (*lmdata)[i].absorberPhotonPosition;
            auto absorberPhotonEnergy = (*lmdata)[i].absorberPhotonEnergy;
            if (CheckEffectedBP(scatterPhotonPosition, scatterPhotonEnergy, absorberPhotonPosition, absorberPhotonEnergy, imageSpacePosition, angleThreshold))
            {
                imgSpace->colors_[j][1] = imgSpace->colors_[j][1]* 4999 / 5000;
                imgSpace->colors_[j][2] = imgSpace->colors_[j][2]* 4999 / 5000;
            }
        }
    }

    return open3d::geometry::PointCloud();
}

