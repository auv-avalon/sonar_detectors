#ifndef DISTANCE_ESTIMATION_HPP_
#define DISTANCE_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"

namespace avalon
{
    
    const static double weightNewValue = 0.4;
    const static double weightOldValue = 0.6;
    const static double max_distance = 0.5;
    const static uint64_t timeout = 1000; //timeout in milliseconds
    
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
        virtual void updateSegment(const avalon::scanSegment& segment);
        double getActualDistance();
        
    private:
        double distance(const base::Vector3d& vec1, const base::Vector3d& vec2) const;
        void checkTimeout();
        
        avalon::obstaclePoint actualPoint;
        double actualDistance;
    };
}

#endif