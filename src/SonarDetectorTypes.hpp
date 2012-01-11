#ifndef SONAR_DETECTOR_TYPES_HPP_
#define SONAR_DETECTOR_TYPES_HPP_

#include <base/eigen.h>
#include <base/time.h>
#include <base/angle.h>

namespace sonar_detectors
{

/**
 * constat for the sonic velocity in water
 */
static const double sonicVelocityInWater = 1500.0;

/**
 * Obstacle point
 */
struct obstaclePoint
{
    base::Vector3d position;
    uint8_t value;
    base::Time time;
    base::Angle angle;
    double distance;
    obstaclePoint()
    : position(0,0,0), value(0), time(base::Time::now()), distance(0){}
};
    
}


#endif