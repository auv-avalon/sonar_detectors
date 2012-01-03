#ifndef SONAR_BEAM_PROCESSING_HPP_
#define SONAR_BEAM_PROCESSING_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"
#include "FeatureExtraction.hpp"

#include <base/samples/sonar_beam.h>
#include <base/samples/laser_scan.h>
#include <base/eigen.h>
#include <base/samples/rigid_body_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <cmath>
#include <avalonmath.h>

namespace sonar_detectors
{
  
class SonarBeamProcessing 
{   
    public:
        SonarBeamProcessing();
        ~SonarBeamProcessing();
        void addSonarEstimation(sonar_detectors::SonarEstimation* estimation);
        void removeSonarEstimation(sonar_detectors::SonarEstimation* estimation);
        void updateSonarData(const base::samples::SonarBeam& sonarScan);
        void updateOrientation(const base::Orientation& orientation);
        void updatePosition(const base::Position& position);
        void setBeamThreshold(double minThreshold);
        void setMinResponseValue(double minValue);
        
    private:
        std::vector<SonarEstimation*> estimators;
        base::Orientation orientation;
        base::Position position;
        double minThreshold;
        sonar_detectors::FeatureExtraction featureExtraction;
};

}

#endif
