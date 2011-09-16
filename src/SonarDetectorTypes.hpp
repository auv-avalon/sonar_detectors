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
 * This will be used to select when entries gets overwritten.
 * persistNewScans means new entries overwrites old entries 
 * with the same angle.
 * persistAll means nothing gets overwritten.
 */
enum PersistMode 
{
    persistNewScans,
    persistAll
};

/**
 * This will be used to select when a estimator gets an 
 * update of the new segment.
 * noSegments means never.
 * forEachEdge means if the beam changes direction.
 * forEachBeam means everytime.
 */
enum SegmentMode
{
    noSegments,
    forEachEdge,
    forEachBeam
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
 * Estimators get updated by scanSegments.
 * A scan segment mainlly contains a point cloud 
 * of obstacle points.
 */
struct scanSegment
{
    std::list<obstaclePoint> pointCloud;
    std::list<obstaclePoint>::iterator latestBeam;
    bool dirty;
    double lastAngle;
    bool risingAngle;
    scanSegment()
    : dirty(false), lastAngle(0), risingAngle(false){}
};

/**
 * Settings for estimators.
 * Relevant angle range and update mode.
 */
struct estimationSettings
{
    double startAngle;
    double endAngle;
    SegmentMode segMode;
    estimationSettings()
    : startAngle(-1), endAngle(-1), segMode(noSegments){}
};
    
}


#endif