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
    estimator est;
    est.estimation = estimation;
    est.settings = estimation->getSettings();
    estimators.push_back(est);
}

void SonarBeamProcessing::removeSonarEstimation(SonarEstimation* estimation)
{
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if (it->estimation == estimation)
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
    obstaclePoint.angle = base::Angle::normalizeRad(scanAngle + base::getYaw(orientation));
    
    return obstaclePoint;
}

void SonarBeamProcessing::updateSonarData(const base::samples::SonarBeam& sonarScan)
{
    double scanAngle = sonarScan.bearing.rad;
    
    //check if angle is required
    bool required = false;
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if (it->settings.startAngle < 0 || it->settings.endAngle < 0 || scanAngle > it->settings.startAngle && scanAngle < it->settings.endAngle)
        {
            required = true;
        }
    }
    if (!required)
        return;
    
    featureExtraction.setBoundingBox(minThreshold, sonarScan.sampling_interval ,sonarScan.speed_of_sound);
    std::vector<float> beam = featureExtraction.noFilter(sonarScan.beam);
    int index = featureExtraction.getFeatureGlobalMaxima(beam);
    
    std::vector<obstaclePoint> obstaclePoints;
    if (index >= 0)
    {
        // calculate feature
        sonar_detectors::obstaclePoint obstaclePoint = computeObstaclePoint(index, sonarScan, orientation);
        obstaclePoints.push_back(obstaclePoint);
    }
    
    //feed estimators
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        // send new features to the estimators if they are in their scan range
        if (it->settings.startAngle < 0 || it->settings.endAngle < 0
            || scanAngle > it->settings.startAngle && scanAngle < it->settings.endAngle)
        {
            it->estimation->updateSegment(obstaclePoints);
        }
    }
    
}

}
