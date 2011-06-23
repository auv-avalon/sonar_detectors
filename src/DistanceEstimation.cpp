#include "DistanceEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include <math.h>

namespace avalon
{

DistanceEstimation::DistanceEstimation()
{
    actualDistance = -1;
}

DistanceEstimation::~DistanceEstimation()
{
    
}

void DistanceEstimation::updateSegment(const avalon::scanSegment& segment)
{
    checkTimeout();
    
    if(!segment.pointCloud.empty() || segment.latestBeam != segment.pointCloud.end())
    {
        base::Position nextPosition(segment.latestBeam->position);
        
        if (actualDistance < 0)
        {
            actualPoint.position = nextPosition;
        }
        else 
        {
            double dist = computeDistance(nextPosition, actualPoint.position);
            if(dist > max_distance)
            {
                // limit distance to the next point
                nextPosition = actualPoint.position + ((nextPosition - actualPoint.position) / dist) * max_distance;
            }
            actualPoint.position = actualPoint.position * weightOldValue + nextPosition * weightNewValue;
        }

        actualPoint.angle = segment.latestBeam->angle;
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