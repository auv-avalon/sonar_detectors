#include "SonarBeamProcessing.hpp"
#include <iostream>

namespace sonar_detectors
{
SonarBeamProcessing::SonarBeamProcessing()
{
    minThreshold = 0;
    position.setZero();
    orientation = Eigen::Quaterniond::Identity();
}

SonarBeamProcessing::~SonarBeamProcessing()
{
    
}

void SonarBeamProcessing::updateOrientation(const base::Orientation& orientation)
{
    this->orientation = orientation;
}

void SonarBeamProcessing::updatePosition(const base::Position& position)
{
    this->position = position;
}

void SonarBeamProcessing::setBeamThreshold(double minThreshold)
{
    this->minThreshold = minThreshold;
}

void SonarBeamProcessing::setMinResponseValue(double minValue)
{
    featureExtraction.setMinResponseValue(minValue);
}

void SonarBeamProcessing::addSonarEstimation(SonarEstimation* estimation)
{
    estimation->setPose(&orientation, &position);
    estimators.push_back(estimation);
}

void SonarBeamProcessing::removeSonarEstimation(SonarEstimation* estimation)
{
    for(std::vector<SonarEstimation*>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if ((*it) == estimation)
        {
            estimators.erase(it);
        }
    }
}

obstaclePoint SonarBeamProcessing::computeObstaclePoint(const int& index, const base::samples::SonarBeam& sonarScan, const base::Orientation& orientation)
{
    double scanAngle = sonarScan.bearing.rad;
    double distance = (double)index * sonarScan.getSpatialResolution();
        
    Eigen::Vector3d wallPoint(distance,0,0);
    Eigen::Vector3d topPoint(0,0,1);
    
    wallPoint = orientation * wallPoint;
    topPoint = orientation * topPoint;
    
    Eigen::AngleAxisd rotate(scanAngle,topPoint);
    
    wallPoint = rotate * wallPoint;
        
    sonar_detectors::obstaclePoint obstaclePoint;
    obstaclePoint.position = wallPoint;
    obstaclePoint.time = sonarScan.time;
    obstaclePoint.value = sonarScan.beam[index];
    obstaclePoint.distance = distance;
    obstaclePoint.angle = base::Angle::fromRad(scanAngle + base::getYaw(orientation));
    
    return obstaclePoint;
}

void SonarBeamProcessing::updateSonarData(const base::samples::SonarBeam& sonarScan)
{
    //check if angle is required
    bool required = false;
    for(std::vector<SonarEstimation*>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if ((*it)->isAngleInRange(sonarScan.bearing))
        {
            required = true;
        }
    }
    if (!required)
        return;
    
    featureExtraction.setBoundingBox(minThreshold, sonarScan.sampling_interval ,sonarScan.speed_of_sound);
    std::vector<float> beam = featureExtraction.convertBeam(sonarScan.beam);
    int index = featureExtraction.getFeatureGlobalMaxima(beam);
    
    std::vector<obstaclePoint> obstaclePoints;
    if (index >= 0)
    {
        // calculate feature
        sonar_detectors::obstaclePoint obstaclePoint = computeObstaclePoint(index, sonarScan, orientation);
        obstaclePoints.push_back(obstaclePoint);
    }
    
    //feed estimators
    for(std::vector<SonarEstimation*>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        (*it)->updateFeatures(obstaclePoints);
    }
    
}

}
