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
        void updateFeature(const base::samples::LaserScan &feature)
        {
            // check if vector is empty
            if(feature.ranges.empty())
                return;
            
            // check if features are out of range
            if(!isAngleInRange(base::Angle::fromRad(feature.start_angle)))
                return;
            
            updateFeatureIntern(feature);
        };
        
        sonar_detectors::estimationSettings getSettings() 
        {
            return settings;
        };
        
        bool isAngleInRange(const base::Angle &angle)
        {
            return (settings.boundedInput ? angle.isInRange(settings.startAngle, settings.endAngle) : true);
        };
        
        void setSettings(const sonar_detectors::estimationSettings& settings)
        {
            this->settings = settings;
        };
         
        void setPose(const base::Orientation* orientation, const base::Position* position)
        {
            this->orientation = orientation;
            this->position = position;
        };
    protected:
        virtual void updateFeatureIntern(const base::samples::LaserScan &feature) = 0;
        
    protected:
        const base::Orientation* orientation;
        const base::Position* position;
        sonar_detectors::estimationSettings settings;
    };
}

#endif