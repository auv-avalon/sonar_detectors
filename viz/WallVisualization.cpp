#include "WallVisualization.hpp"
#include <osg/Geode>
#include <sonar_detectors/SonarDetectorMath.hpp>

namespace vizkit
{
    
WallVisualization::WallVisualization()
{
    wall_length = 20.0;
}

/**
 * Creates the main node and attachs the wall model.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > WallVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up wall geometry
    wallGeom = new osg::Geometry;
    wallOSG = new osg::Vec3Array;
    wallGeom->setVertexArray(wallOSG);
    osg::Vec4Array* wallcolor = new osg::Vec4Array;
    wallcolor->push_back(osg::Vec4(0.0f, 0.8f, 0.8f, 0.8f));
    wallGeom->setColorArray(wallcolor);
    wallGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(wallGeom.get());
    
    mainNode->addChild(geode);
    
    return mainNode;
}

void WallVisualization::updateDataIntern(const std::vector<base::Vector3d>& data)
{
    wall = data;
}

void WallVisualization::updateMainNode(osg::Node* node)
{
    wallOSG->clear();
    wallGeom->removePrimitiveSet(0, wallGeom->getNumPrimitiveSets());
    if (!wall.empty() && wall.size() % 2 == 0)
    {
        for(unsigned int i = 0; i + 1 < wall.size(); i+=2)
        {
            if(wall[i] == wall[i+1] || wall[i+1] == base::Vector3d(0,0,0))
                continue;
            base::Vector3d direction_vector = wall[i+1] * ((wall_length * 0.5) / sonar_detectors::length(wall[i+1]));
            base::Vector3d pos1 = wall[i] - direction_vector;
            base::Vector3d pos2 = wall[i] + direction_vector;
            osg::Vec3d vec1(pos1.x(), pos1.y(), pos1.z());
            osg::Vec3d vec2(pos2.x(), pos2.y(), pos2.z());
            wallOSG->push_back(vec1 + osg::Vec3d(0,0,1));
            wallOSG->push_back(vec1 + osg::Vec3d(0,0,-1));
            wallOSG->push_back(vec2 + osg::Vec3d(0,0,1));
            wallOSG->push_back(vec2 + osg::Vec3d(0,0,-1));
        }
        
        if(wallOSG->size() >= 4)
        {
            wallGeom->setVertexArray(wallOSG);
            for(unsigned int i = 0; i < wallOSG->size() - 3; i += 4)
            {
                osg::ref_ptr<osg::DrawElementsUInt> drawWall = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
                drawWall->push_back(i);
                drawWall->push_back(i+1);
                drawWall->push_back(i+3);
                drawWall->push_back(i+2);
                wallGeom->addPrimitiveSet(drawWall);
            }
        }
    }
}

}