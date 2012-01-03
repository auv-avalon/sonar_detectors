#include "DistanceEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include <cmath>

namespace sonar_detectors
{

DistanceEstimation::DistanceEstimation()
{
    actualFeature.ranges.front() = base::samples::MEASUREMENT_ERROR;
}

DistanceEstimation::~DistanceEstimation()
{
    
}

void DistanceEstimation::updateFeatureIntern(const base::samples::LaserScan &feature)
{
    checkTimeout();
    
    if(feature.isValidBeam(0) && feature.ranges.front() > base::samples::MAX_RANGE_ERROR)
    {
        if(actualFeature.ranges.front() <= base::samples::MAX_RANGE_ERROR)
        {
            actualFeature.ranges.front() = feature.ranges.front();
        }
        else 
        {
            double new_range = (double)feature.ranges.front() * 0.001;
            double actual_range = (double)actualFeature.ranges.front() * 0.001;
            double dist = abs(new_range - actual_range);
            if(dist > max_distance)
            {
                // limit distance to the next point
                new_range = actual_range + ((new_range - actual_range) / dist) * max_distance;
            }
            actual_range = actual_range * weightOldValue + new_range * weightNewValue;
            actualFeature.ranges.front() = (uint32_t)(actual_range * 1000.0);
        }

        actualFeature.start_angle = feature.start_angle;
        actualFeature.time = feature.time;
    }
}

void DistanceEstimation::checkTimeout()
{
    if ((base::Time::now().toMilliseconds() - actualFeature.time.toMilliseconds()) > timeout)
    {
        actualFeature.ranges.front() = base::samples::MEASUREMENT_ERROR;
    }
}

double DistanceEstimation::getActualDistance()
{
    checkTimeout();
    if(actualFeature.ranges.front() <= base::samples::MAX_RANGE_ERROR)
        return -1;
    return (double)actualFeature.ranges.front() * 0.001;
}

}