#include "SonarObstaclesVisualization.hpp"

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <iostream>
#include <vizkit3d/ColorConversionHelper.hpp>

using namespace vizkit3d;

vizkit3d::SonarObstaclesVisualization::SonarObstaclesVisualization()
    : mYForward(false),colorize(false),show_polygon(true),colorize_interval(0.2)
{
    scanOrientation = Eigen::Quaterniond::Identity();
    scanPosition.setZero();
}

vizkit3d::SonarObstaclesVisualization::~SonarObstaclesVisualization()
{
}

void vizkit3d::SonarObstaclesVisualization::updateDataIntern(const sonar_detectors::ObstacleFeatures& data)
{
    scan = data;
}

void vizkit3d::SonarObstaclesVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    scanOrientation = data.orientation;
    scanPosition = data.position;
}

osg::ref_ptr< osg::Node > vizkit3d::SonarObstaclesVisualization::createMainNode()
{
    transformNode = new osg::PositionAttitudeTransform();
    scanNode = new osg::Geode();
    transformNode->addChild(scanNode);

    scanGeom = new osg::Geometry();

    //setup normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    scanGeom->setNormalArray(normals);
    scanGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    //set size
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(5.0);
    point->setDistanceAttenuation( osg::Vec3(1.0, 1.0, 1.0 ) );
    point->setMinSize( 3.0 );
    point->setMaxSize( 5.0 );
    scanGeom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );

    //turn on transparacny
    scanNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    scanNode->addDrawable(scanGeom);
    
    return transformNode;
}

void vizkit3d::SonarObstaclesVisualization::updateMainNode(osg::Node* node)
{
    transformNode->setPosition(osg::Vec3d(scanPosition.x(), scanPosition.y(), scanPosition.z()));
    transformNode->setAttitude( osg::Quat(scanOrientation.x(), scanOrientation.y(), scanOrientation.z(), scanOrientation.w()));
    
    osg::Vec3Array *scanVertices = new osg::Vec3Array();

    std::vector<Eigen::Vector3d> points;
    
    for(std::vector<sonar_detectors::ObstacleFeature>::iterator it = scan.features.begin(); it != scan.features.end(); it++){
      
      Eigen::Vector3d p;
      p.x() = (it->range / 1000.0) * std::cos(scan.angle);
      p.y() = (it->range / 1000.0) * std::sin(scan.angle);
      p.z() = 0.0;
      points.push_back(p);
    }
    
    if (mYForward)
    {
        base::Quaterniond rot;
        rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
        for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it != points.end(); it++)
            *it = rot * (*it);
    }

    if(colorize)    
    {
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(0.0,0.0,0.0,0.0));
        float col;
        int interval = colorize_interval*1000;
        double angle = scan.angle;
        for(std::vector<sonar_detectors::ObstacleFeature>::iterator it = scan.features.begin(); it != scan.features.end(); it++)
        {
            if(it->range > 0.0)
            {
                col  = ((float)(((int)(it->range * cos(angle))%interval)))/interval;
                osg::Vec4 color( 1.0, 1.0, 1.0, 1.0 );
                hslToRgb(col, 1.0, 0.5, color.r(), color.g(), color.b());
                colors->push_back(color);
            }
        }
        scanGeom->setColorArray(colors.get());
        scanGeom->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }
    else
    {
        osg::Vec4Array *colors = new osg::Vec4Array();
        colors->push_back(osg::Vec4(0,0,0.3,0.5));
        colors->push_back(osg::Vec4(1,0,0,1));
        scanGeom->setColorArray(colors);
        scanGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    }

    scanVertices->reserve(points.size()+1);
    scanVertices->push_back(osg::Vec3(0,0,0));

    for(std::vector<Eigen::Vector3d>::const_iterator it = points.begin(); it != points.end(); it++)
	scanVertices->push_back(osg::Vec3d(it->x(), it->y(), it->z()));
    scanGeom->setVertexArray(scanVertices);

    while(!scanGeom->getPrimitiveSetList().empty())
	scanGeom->removePrimitiveSet(0);
   
    if(show_polygon)
        scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,scanVertices->size()));
    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scanVertices->size()));
}

//display only points  
osg::ref_ptr<osg::Node> SonarObstaclesVisualization::cloneCurrentViz()
{
    //copy hole scene graph and remove PrimitiveSet polygon
    osg::ref_ptr<osg::Group> group = VizPluginBase::cloneCurrentViz()->asGroup();
    osg::ref_ptr<osg::Node> node = group->getChild(0);
    if(!node)
        return group;
    osg::ref_ptr<osg::Group> group2 = node->asGroup();
    if(!group2)
        return group;
    node = group2->getChild(0);
    if(!node)
        return group;
    osg::ref_ptr<osg::Geode> geode = node->asGeode();
    if(!geode)
        return group;
    osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(0);
    if(!drawable)
        return group;
    osg::ref_ptr<osg::Geometry> geometry = drawable->asGeometry();
    if(!geometry)
        return group;
    if(geometry->getNumPrimitiveSets() == 2)
        geometry->removePrimitiveSet(0);
    return group;
}

bool SonarObstaclesVisualization::isYForwardModeEnabled() const { return mYForward; }
void SonarObstaclesVisualization::setYForwardMode(bool enabled) { mYForward = enabled; }

void SonarObstaclesVisualization::setColorize(bool value){colorize = value;emit propertyChanged("Colorize");}
bool SonarObstaclesVisualization::isColorizeEnabled()const { return colorize; }

void SonarObstaclesVisualization::setShowPolygon(bool value){show_polygon = value;emit propertyChanged("ShowPolygon");}
bool SonarObstaclesVisualization::isShowPolygonEnabled()const { return show_polygon; }

void SonarObstaclesVisualization::setColorizeInterval(double value){colorize_interval = value;emit propertyChanged("ColorizeInterval");}
double SonarObstaclesVisualization::getColorizeInterval()const { return colorize_interval; }
