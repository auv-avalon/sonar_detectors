#ifndef SONAR_DETECTOR_TYPES_HPP_
#define SONAR_DETECTOR_TYPES_HPP_

#include <base/samples/sonar_scan.h>
#include <base/eigen.h>
#include <base/samples/rigid_body_state.h>

#include <list>
#include <vector>

namespace avalon{
    
enum BeamMode 
{
    globalMaximum,
    firstLokalMaximum,
    allLokalMaxima,
    allEntries
};

enum PersistMode 
{
    persistNewScans,
    persistAll
};

enum SegmentMode
{
    noSegments,
    forEachEdge,
    forEachBeam
};

static const double sonicVelocityInWater = 1500.0;

struct obstaclePoint
{
    base::Position position;
    base::samples::SonarScan::uint8_t value;
    base::Time time;
    double angle;
    obstaclePoint()
    : position(0,0,0), value(0), angle(0){}
};

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