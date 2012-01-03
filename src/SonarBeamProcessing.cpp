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
    
    // calculate feature
    base::samples::LaserScan feature = sonar_detectors::FeatureExtraction::computeLaserScan(index, sonarScan);
    
    //feed estimators
    for(std::vector<SonarEstimation*>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        (*it)->updateFeature(feature);
    }
    
}

}
