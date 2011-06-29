#ifndef SONAR_BEAM_PROCESSING_HPP_
#define SONAR_BEAM_PROCESSING_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"

#include <base/samples/sonar_scan.h>
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
        SonarBeamProcessing(BeamMode beamMode, PersistMode persistMode);
        ~SonarBeamProcessing();
        void addSonarEstimation(avalon::SonarEstimation* estimation);
        void removeSonarEstimation(avalon::SonarEstimation* estimation);
        void updateSonarData(const base::samples::SonarScan& sonarScan);
        void updateOrientation(const base::Orientation& orientation);
        void updatePosition(const base::Position& position);
        void selectBeamMode(BeamMode mode);
        void selectPersistMode(PersistMode mode);
        void enableBeamThreshold(bool b);
        void setBeamThreshold(double minThreshold, double maxThreshold);
        void setMinResponseValue(int minValue);
        
    private:
        std::vector<estimator> estimators;
        base::Orientation orientation;
        base::Position position;
        BeamMode beamMode;
        PersistMode persistMode;
        bool enableThreshold;
        double minThreshold;
        double maxThreshold;
        int minResponseValue;
        int indexWindowSize;
        
        std::vector<int> computeSonarScanIndex(const std::vector<base::samples::SonarScan::uint8_t>& scan, const int& minIndex, const int& maxIndex, const int& minValue);
        int getNextMaximum(const int& startIndex, const int& endIndex, const int& minValue, const std::vector<base::samples::SonarScan::uint8_t>& scan);
        avalon::obstaclePoint computeObstaclePoint(const int& index, const base::samples::SonarScan& sonarScan);
        void persistPoints(const std::vector<obstaclePoint>& obstaclePoints, const double& angle, scanSegment& segment);
        bool isSegmentDone(avalon::estimator& estimator, const double& angle);
};

}

#endif