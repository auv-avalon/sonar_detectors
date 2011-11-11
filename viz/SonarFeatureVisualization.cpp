#include "SonarFeatureVisualization.hpp"
#include <osg/Geode>

namespace vizkit
{

SonarFeatureVisualization::SonarFeatureVisualization()
{
    VizPluginRubyAdapter(SonarFeatureVisualization, base::samples::Pointcloud, PointCloud)
    VizPluginRubyAdapter(SonarFeatureVisualization, std::vector< base::Vector3d >, ChannelData)
    newPoints = false;
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
    color = new osg::Vec4Array;
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarFeatureVisualization::updateDataIntern(const base::samples::Pointcloud& data)
{
    pointCloud = data;
    newPoints = true;
}

void SonarFeatureVisualization::updateDataIntern(const std::vector< base::Vector3d >& data)
{
    channelInfos = data;
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
    if(newPoints)
    {
        newPoints = false;
        pointsOSG->clear();
        color->clear();
        std::vector<base::Vector3d>::const_iterator chan_pos = channelInfos.begin();
        for(std::vector<base::Vector3d>::const_iterator pos = pointCloud.points.begin(); pos != pointCloud.points.end(); pos++)
        {
            osg::Vec3d vec(pos->x(), pos->y(), pos->z());
            pointsOSG->push_back(vec);
            pointsOSG->push_back(vec + osg::Vec3d(0,0,2));
            
            if(chan_pos != channelInfos.end())
            {
                color->push_back(osg::Vec4(chan_pos->x(), chan_pos->y(), chan_pos->z(), 1.0f));
                chan_pos++;
            }
            else
            {
                color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
            }
        }
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
    }
}

VizkitQtPlugin(SonarFeatureVisualization)
}