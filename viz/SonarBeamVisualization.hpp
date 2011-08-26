#ifndef sonar_detectors_SonarBeamVisualization_H
#define sonar_detectors_SonarBeamVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/VizPlugin.hpp>
#include <osg/Geode>

namespace vizkit
{
    class SonarBeamVisualization
        : public vizkit::VizPlugin<base::samples::SonarScan>
        , boost::noncopyable
    {
    public:
        SonarBeamVisualization();
        ~SonarBeamVisualization();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(base::samples::SonarScan const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
