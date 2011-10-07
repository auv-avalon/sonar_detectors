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
        virtual void updateSegment(const std::vector<sonar_detectors::obstaclePoint> &features) = 0;
        
        sonar_detectors::estimationSettings getSettings() 
        {
            return settings;
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
    
    struct estimator
    {
        SonarEstimation* estimation;
        estimationSettings settings;
    };
}

#endif