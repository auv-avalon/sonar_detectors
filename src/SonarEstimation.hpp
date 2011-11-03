#ifndef SONAR_ESTIMATION_HPP_
#define SONAR_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include <base/samples/rigid_body_state.h>
#include <vector>

namespace sonar_detectors
{
    class SonarEstimation
    {
    public:
        virtual void updateFeaturesIntern(const std::vector<sonar_detectors::obstaclePoint> &features) = 0;
        
        void updateFeatures(const std::vector<sonar_detectors::obstaclePoint> &features)
        {
            // check if vector is empty
            if(features.empty())
                return;
            
            // check if features are out of range
            if(!isAngleInRange(features.front().angle))
                return;
            
            updateFeaturesIntern(features);
        };
        
        sonar_detectors::estimationSettings getSettings() 
        {
            return settings;
        };
        
        bool isAngleInRange(base::Angle angle)
        {
            return (settings.boundedInput ? angle.isInRange(settings.startAngle, settings.endAngle) : true);
        };
        
        void setSettings(sonar_detectors::estimationSettings& settings)
        {
            this->settings = settings;
        };
         
        void setPose(const base::Orientation* orientation, const base::Position* position)
        {
            this->orientation = orientation;
            this->position = position;
        };
    protected:
        const base::Orientation* orientation;
        const base::Position* position;
        sonar_detectors::estimationSettings settings;
    };
}

#endif