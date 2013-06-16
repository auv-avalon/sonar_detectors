#include "SonarDepthMapVisualization.hpp"
#include <osg/Geode>

namespace vizkit
{

SonarDepthMapVisualization::SonarDepthMapVisualization()
{
    newPoints = false;
    default_feature_color = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > SonarDepthMapVisualization::createMainNode()
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

QColor SonarDepthMapVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void SonarDepthMapVisualization::setDefaultFeatureColor(QColor color)
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
void SonarDepthMapVisualization::updateDataIntern(const base::samples::Pointcloud& data)
{
    pointCloud = data;
    newPoints = true;
}

void SonarDepthMapVisualization::updateDataIntern(const std::vector< base::Vector3d >& data)
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
/*
void SonarDepthMapVisualization::updateMainNode(osg::Node* node)
{
    if(newPoints)
    {
        newPoints = false;
        pointsOSG->clear();
        color->clear();
        std::vector<base::Vector3d>::const_iterator chan_pos = channelInfos.begin();
        for(std::vector<base::Vector3d>::const_iterator pos = pointCloud.points.begin(); pos != pointCloud.points.end(); pos++)
        {
            osg::Vec3d vec((pos->x()-0.1) * 0.1, pos->y() * 0.1, pos->z() * 0.1);
            pointsOSG->push_back(vec);
            pointsOSG->push_back(osg::Vec3d((pos->x()+0.1)* 0.1, pos->y() * 0.1, pos->z() * 0.1));
	    
	    osg::Vec3d vec2(pos->x() * 0.1, (pos->y()-0.1) * 0.1, pos->z() * 0.1);
            pointsOSG->push_back(vec2);
            pointsOSG->push_back(osg::Vec3d(pos->x() * 0.1, (pos->y()+0.1) * 0.1, pos->z() * 0.1));
            
	    //color->push_back(default_feature_color);
	    //color->push_back(default_feature_color);	
	    
	    
	    color->push_back(osg::Vec4f((float)std::fabs( pos->z() * 255/30), (float)std::fabs(30 * 255/ pos->z()), 0.0f, 254.0f  ));
	    color->push_back(osg::Vec4f((float)std::fabs( pos->z() * 255/30), (float)std::fabs(30 * 255/ pos->z()), 0.0f, 254.0f  ));
            
        }
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
    }
}*/

void SonarDepthMapVisualization::updateMainNode(osg::Node* node)
{
    if(newPoints)
    {
      
       newPoints = false;
        pointsOSG->clear();
        color->clear();

	for(std::vector<base::Vector3d>::const_iterator pos = pointCloud.points.begin(); pos != pointCloud.points.end(); pos++)
        {
	  
	  base::Vector3d right,up;
	  double foundRight = false, foundUp = false;
	  
	  for(std::vector<base::Vector3d>::const_iterator n = pointCloud.points.begin(); n != pointCloud.points.end(); n++){
	    
	    if(*n!=*pos &&  n->y() == pos->y() && n->x() > pos->x() && ( (!foundRight) || (foundRight && n->x() < right.x()))){
	     right = *n;
	     foundRight = true;
	    }
	    
	    if(*n!=*pos &&  n->x() == pos->x() && n->y() > pos->y() && ( (!foundUp) || (foundUp && n->y() < up.y()))){
	     up = *n;
	     foundUp = true;
	    } 
	    
	  }
	  
	  if(foundRight){
	    pointsOSG->push_back(osg::Vec3d(pos->x() * 0.1, pos->y() * 0.1, pos->z() * 0.1));
	    pointsOSG->push_back(osg::Vec3d(right.x() * 0.1, right.y() * 0.1, right.z() * 0.1));
	    color->push_back(default_feature_color);
	    
	  }
	  
	  if(foundUp){
	    pointsOSG->push_back(osg::Vec3d(pos->x() * 0.1, pos->y() * 0.1, pos->z() * 0.1));
	    pointsOSG->push_back(osg::Vec3d(up.x() * 0.1, up.y() * 0.1, up.z() * 0.1));
	    color->push_back(default_feature_color);
	    
	  }
	  
	  if(!foundRight && !foundUp){
	    osg::Vec3d vec((pos->x()-0.1) * 0.1, pos->y() * 0.1, pos->z() * 0.1);
            pointsOSG->push_back(vec);
            pointsOSG->push_back(osg::Vec3d((pos->x()+0.1)* 0.1, pos->y() * 0.1, pos->z() * 0.1));
	    
	    osg::Vec3d vec2(pos->x() * 0.1, (pos->y()-0.1) * 0.1, pos->z() * 0.1);
            pointsOSG->push_back(vec2);
            pointsOSG->push_back(osg::Vec3d(pos->x() * 0.1, (pos->y()+0.1) * 0.1, pos->z() * 0.1));

	    color->push_back(default_feature_color);
	    color->push_back(default_feature_color);
	    
	  }  
	  
	}	
	
	drawArrays->setCount(pointsOSG->size());
	pointGeom->setVertexArray(pointsOSG);
	pointGeom->setColorArray(color);
    }
   
}

}