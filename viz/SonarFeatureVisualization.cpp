#include "SonarFeatureVisualization.hpp"
#include <osg/Geode>

namespace vizkit
{

SonarFeatureVisualization::SonarFeatureVisualization()
{
    VizPluginRubyAdapter(SonarFeatureVisualization, base::samples::Pointcloud, PointCloud)
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > SonarFeatureVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    osg::Vec4Array* color = new osg::Vec4Array;
    color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;
}

/**
 * @return plugin name
 */
const std::string SonarFeatureVisualization::getPluginName() const
{
    return "SonarFeatureVisualization";
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarFeatureVisualization::updateDataIntern(const base::samples::Pointcloud& data)
{
    pointCloud = data;
}

/**
 * Main callback method of osg to update all drawings.
 * The method updates the position if there is a new one and
 * adds new sonar beams if there are new ones. If a wall 
 * estimatior is connected the position of the wall will 
 * be updated too.
 * 
 * @param node osg main node
 */
void SonarFeatureVisualization::updateMainNode(osg::Node* node)
{
    pointsOSG->clear();
    for(std::vector<base::Vector3d>::const_iterator pos = pointCloud.points.begin(); pos != pointCloud.points.end(); pos++)
    {
        osg::Vec3d vec(pos->x(), pos->y(), pos->z());
        pointsOSG->push_back(vec);
        pointsOSG->push_back(vec + osg::Vec3d(0,0,2));
    }
    drawArrays->setCount(pointsOSG->size());
    pointGeom->setVertexArray(pointsOSG);
}

VizkitQtPlugin(SonarFeatureVisualization)
}