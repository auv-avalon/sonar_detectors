#ifndef SONAR_ESTIMATION_HPP_
#define SONAR_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include <base/samples/rigid_body_state.h>

namespace avalon
{
    class SonarEstimation
    {
    public:
        virtual void updateSegment(const avalon::scanSegment& segment) = 0;
        
        avalon::estimationSettings getSettings() 
        {
            return settings;
        };
        
        void setSettings(avalon::estimationSettings& settings)
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
        avalon::estimationSettings settings;
    };
    
    struct estimator
    {
        SonarEstimation* estimation;
        estimationSettings settings;
        scanSegment segment;
    };
}

#endif