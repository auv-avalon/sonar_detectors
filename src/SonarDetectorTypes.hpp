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
  

struct ObstacleFeature
{
  uint32_t range;
  double confidence;  
};

struct ObstacleFeatures
{
  base::Time time;
  double angle;
  std::vector<ObstacleFeature> features;
 
};

  struct SonarFeature{
    base::Vector2d position;
    base::Vector2d span;
    double avg_confidence;
    double sum_confidence;
    double confidence;
    int number_of_cells;
    
    /**
     * < operator for sorting of features. ATTENTION: operator is reversed, to achive a reversed sorting
     */
    bool operator< ( SonarFeature const& rhs) const
    { 
      return ( confidence > rhs.confidence  ); 
    }    
    
    
  };
  
  struct SonarFeatures{
    
    base::Time time;
    std::vector<SonarFeature> features;
    
  };


}


#endif