#ifndef SONAR_DETECTOR_TYPES_HPP_
#define SONAR_DETECTOR_TYPES_HPP_

#include <base/eigen.h>
#include <base/time.h>

#include <list>
#include <vector>

namespace avalon{
    
/**
 * This will be used to select the major entries inside 
 * a single beam.
 */
enum BeamMode 
{
    globalMaximum,
    firstLokalMaximum,
    allLokalMaxima,
    allEntries
};

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
    double angle;
    obstaclePoint()
    : position(0,0,0), value(0), time(base::Time::now()), angle(0){}
};

/**
 * Settings for estimators.
 * Relevant angle range and update mode.
 */
struct estimationSettings
{
    double startAngle;
    double endAngle;
    estimationSettings()
    : startAngle(-1), endAngle(-1){}
};
    
}


#endif