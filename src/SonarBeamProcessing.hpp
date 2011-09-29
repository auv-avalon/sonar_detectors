#ifndef SONAR_BEAM_PROCESSING_HPP_
#define SONAR_BEAM_PROCESSING_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"
#include "FeatureExtraction.hpp"

#include <base/samples/sonar_beam.h>
#include <base/eigen.h>
#include <base/samples/rigid_body_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cmath>
#include <avalonmath.h>

namespace avalon
{
  
class SonarBeamProcessing 
{   
    public:
        SonarBeamProcessing();
        ~SonarBeamProcessing();
        void addSonarEstimation(avalon::SonarEstimation* estimation);
        void removeSonarEstimation(avalon::SonarEstimation* estimation);
        void updateSonarData(const base::samples::SonarBeam& sonarScan);
        void updateOrientation(const base::Orientation& orientation);
        void updatePosition(const base::Position& position);
        void setBeamThreshold(double minThreshold);
        void setMinResponseValue(double minValue);
        
        static avalon::obstaclePoint computeObstaclePoint(const int& index, const base::samples::SonarBeam& sonarScan, const base::Orientation& orientation);
        
    private:
        std::vector<estimator> estimators;
        base::Orientation orientation;
        base::Position position;
        double minThreshold;
        sonar_detectors::FeatureExtraction featureExtraction;
};

}

#endif
