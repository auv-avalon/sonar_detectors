#include "SonarBeamVisualization.hpp"

using namespace vizkit;

struct SonarBeamVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::SonarScan data;
};


SonarBeamVisualization::SonarBeamVisualization()
    : p(new Data)
{
}

SonarBeamVisualization::~SonarBeamVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> SonarBeamVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void SonarBeamVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void SonarBeamVisualization::updateDataIntern(base::samples::SonarScan const& value)
{
    p->data = value;
}

