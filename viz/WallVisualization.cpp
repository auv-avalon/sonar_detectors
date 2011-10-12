#include "WallVisualization.hpp"
#include <osg/Geode>

namespace vizkit
{
    
WallVisualization::WallVisualization()
{
    VizPluginRubyAdapter(WallVisualization, std::vector<base::Vector3d>, WallData)
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
    wallcolor->push_back(osg::Vec4(0.0f, 0.8f, 0.8f, 1.0f));
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
    if (wall.size() == 2 && wall.front() != wall.back())
    {
        base::Vector3d pos1 = wall.front() - wall.back() * 5;
        base::Vector3d pos2 = wall.front() + wall.back() * 5;
        osg::Vec3d vec1(pos1.x(), pos1.y(), pos1.z());
        osg::Vec3d vec2(pos2.x(), pos2.y(), pos2.z());
        wallOSG->push_back(vec1 + osg::Vec3d(0,0,1));
        wallOSG->push_back(vec1 + osg::Vec3d(0,0,-1));
        wallOSG->push_back(vec2 + osg::Vec3d(0,0,1));
        wallOSG->push_back(vec2 + osg::Vec3d(0,0,-1));
        
        wallGeom->removePrimitiveSet(0, wallGeom->getNumPrimitiveSets());
        if(wallOSG->size() >= 4) 
        {
            wallGeom->setVertexArray(wallOSG);
            for(int i = 0; i < wallOSG->size() - 3; i += 4)
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
    else
    {
        wallGeom->removePrimitiveSet(0, wallGeom->getNumPrimitiveSets());
    }
}

VizkitQtPlugin(WallVisualization)
}