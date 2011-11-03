#include "DistanceEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include <math.h>

namespace sonar_detectors
{

DistanceEstimation::DistanceEstimation()
{
    actualDistance = -1;
}

DistanceEstimation::~DistanceEstimation()
{
    
}

void DistanceEstimation::updateFeaturesIntern(const std::vector<sonar_detectors::obstaclePoint> &features)
{
    checkTimeout();
    
    if (!features.empty())
    {
        obstaclePoint nextPosition = features.front();
        
        if (actualDistance < 0)
        {
            actualPoint.position = nextPosition.position;
        }
        else 
        {
            double dist = computeDistance(nextPosition.position, actualPoint.position);
            if(dist > max_distance)
            {
                // limit distance to the next point
                nextPosition.position = actualPoint.position + ((nextPosition.position - actualPoint.position) / dist) * max_distance;
            }
            actualPoint.position = actualPoint.position * weightOldValue + nextPosition.position * weightNewValue;
        }

        actualPoint.angle = nextPosition.angle;
        actualPoint.time = base::Time::now();
        actualDistance = computeDistance(*position, actualPoint.position);
    }
}

void DistanceEstimation::checkTimeout()
{
    if ((base::Time::now().toMilliseconds() - actualPoint.time.toMilliseconds()) > timeout)
    {
        actualDistance = -1;
    }
}

double DistanceEstimation::getActualDistance()
{
    checkTimeout();
    return actualDistance;
}

}