#ifndef SONAR_ESTIMATION_HPP_
#define SONAR_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include <base/samples/rigid_body_state.h>
#include <base/samples/laser_scan.h>
#include <vector>

namespace sonar_detectors
{
    class SonarEstimation
    {
    public:
        void updateFeature(const base::samples::LaserScan &feature, const base::Angle &global_heading = base::Angle::fromRad(0.0))
        {
            // check if vector is empty
            if(feature.ranges.empty())
                return;
            
            // check if the LaserScan contains only ranges in the same angle
            if(feature.angular_resolution != 0.0)
            {
                throw std::runtime_error("The LaserScan must contain only features in the same angle. The angular resolution should be 0.");
            }
            
            // set global angles
            this->global_heading = global_heading;
            this->global_start_angle = start_angle + global_heading;
            this->global_end_angle = end_angle + global_heading;
            if(global_heading.rad != 0.0)
            {
                base::samples::LaserScan global_feature;
                global_feature.angular_resolution = feature.angular_resolution;
                global_feature.maxRange = feature.maxRange;
                global_feature.minRange = feature.minRange;
                global_feature.ranges = feature.ranges;
                global_feature.remission = feature.remission;
                global_feature.speed = feature.speed;
                global_feature.time = feature.time;
                global_feature.start_angle = base::Angle::normalizeRad(feature.start_angle + global_heading.rad);
                updateFeatureIntern(global_feature);
            }
            else
            {
                updateFeatureIntern(feature);
            }
        };
        
        void setEstimationZone(const base::Angle &start_angle, const base::Angle &end_angle)
        {
            this->start_angle = start_angle;
            this->end_angle = end_angle;
        };
        
    protected:
        SonarEstimation()
        {
            global_heading = base::Angle::fromRad(0.0);
            global_start_angle = base::Angle::fromRad(0.0);
            global_end_angle = base::Angle::fromRad(0.0);
            start_angle = base::Angle::fromRad(0.0);
            end_angle = base::Angle::fromRad(0.0);
        };
        
        virtual void updateFeatureIntern(const base::samples::LaserScan &feature) = 0;
        
    protected:
        base::Angle global_heading;
        base::Angle global_start_angle;
        base::Angle global_end_angle;
    private:
        base::Angle start_angle;
        base::Angle end_angle;
    };
}

#endif