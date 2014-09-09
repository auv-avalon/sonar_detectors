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
     * This class tries to estimate the current distance to an object.
     */
    class DistanceEstimation : public SonarEstimation
    {
    public:
        DistanceEstimation();
        ~DistanceEstimation();
        double getActualDistance();
        
    protected:
        virtual void updateFeatureIntern(const base::samples::LaserScan &feature, const Eigen::Affine3d &featureInOdometry);
        
    private:
        void checkTimeout();
        
        base::samples::LaserScan actualFeature;
    };
}

#endif