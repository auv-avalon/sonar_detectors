#include "DistanceEstimation.hpp"
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
            double dist = distance(nextPosition, actualPoint.position);
            if(dist > max_distance)
            {
                // limit distance to the next point
                nextPosition = actualPoint.position + ((nextPosition - actualPoint.position) / dist) * max_distance;
            }
            actualPoint.position = actualPoint.position * weightOldValue + nextPosition * weightNewValue;
        }

        actualPoint.angle = segment.latestBeam->angle;
        actualPoint.time = base::Time::now();
        actualDistance = distance(*position, actualPoint.position);
    }
}

double DistanceEstimation::distance(const base::Vector3d& vec1, const base::Vector3d& vec2) const
{
    return sqrt(pow(vec1.x()-vec2.x(), 2) + pow(vec1.y()-vec2.y(), 2) + pow(vec1.z()-vec2.z(), 2));
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