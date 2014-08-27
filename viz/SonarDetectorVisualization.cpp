#include "SonarDetectorVisualization.hpp"
#include <osg/Geode>

namespace vizkit3d
{

SonarDetectorVisualization::SonarDetectorVisualization()
{
    newPoints = false;
    default_feature_color = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > SonarDetectorVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX); //PRIMITIVE_SET);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::QUADS, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;
}

QColor SonarDetectorVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void SonarDetectorVisualization::setDefaultFeatureColor(QColor color)
{
    default_feature_color.x() = color.redF();
    default_feature_color.y() = color.greenF();
    default_feature_color.z() = color.blueF();
    default_feature_color.w() = color.alphaF();
    emit propertyChanged("defaultFeatureColor");
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarDetectorVisualization::updateDataIntern(const sonar_detectors::SonarFeatures& data)
{
    features = data;
    newPoints = true;
}

void SonarDetectorVisualization::updateDataIntern(const std::vector< base::Vector3d >& data)
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


void SonarDetectorVisualization::updateMainNode(osg::Node* node)
{
    if(newPoints)
    {
        newPoints = false;
        pointsOSG->clear();
        color->clear();
        
        for(std::vector<sonar_detectors::SonarFeature>::iterator it = features.features.begin(); it != features.features.end(); it++){
          
          base::Vector2d pos = it->position;
          base::Vector2d span = it->span * 0.5;
          
          pointsOSG->push_back(osg::Vec3(pos.x() + span.x() , pos.y() + span.y() , -0.1));
          pointsOSG->push_back(osg::Vec3(pos.x() - span.x() , pos.y() + span.y() , -0.1));
          pointsOSG->push_back(osg::Vec3(pos.x() - span.x() , pos.y() - span.y() , -0.1));
          pointsOSG->push_back(osg::Vec3(pos.x() + span.x() , pos.y() - span.y() , -0.1));
          color->push_back( osg::Vec4f( 0.0, 0.0, it->confidence ,1.0) );
          color->push_back( osg::Vec4f( 0.0, 0.0, it->confidence ,1.0) );
          color->push_back( osg::Vec4f( 0.0, 0.0, it->confidence ,1.0) );
          color->push_back( osg::Vec4f( 0.0, 0.0, it->confidence ,1.0) );
          
        }        
        
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);        
        
    
    }
   
}

}
