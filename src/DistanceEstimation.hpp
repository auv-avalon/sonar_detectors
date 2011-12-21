#ifndef DISTANCE_ESTIMATION_HPP_
#define DISTANCE_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"

namespace sonar_detectors
{
    
    const static double weightNewValue = 0.4;
    const static double weightOldValue = 0.6;
    const static double max_distance = 0.5;
    const static int64_t timeout = 1000; //timeout in milliseconds
    
    /**
     * This class trys to estimate the current distance to an object.
     * 
     * estimationSettings should be:
     * - angle range less than PI, lesser is better
     * - SegmentMode: forEachBeam
     */
    class DistanceEstimation : public SonarEstimation
    {
    public:
        DistanceEstimation();
        ~DistanceEstimation();
        virtual void updateFeaturesIntern(const std::vector<sonar_detectors::obstaclePoint> &features);
        double getActualDistance();
        
    private:
        void checkTimeout();
        
        sonar_detectors::obstaclePoint actualPoint;
        double actualDistance;
    };
}

#endif