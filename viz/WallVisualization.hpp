#ifndef WallVisualization_H
#define WallVisualization_H

#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/eigen.h>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit
{
    
class WallVisualization : public vizkit::Vizkit3DPlugin< std::vector<base::Vector3d> >
{
public:
    WallVisualization();
    const std::string getPluginName() const;
    
protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode( osg::Node* node );
    void updateDataIntern ( const std::vector<base::Vector3d>& data );
    
private:
    std::vector<base::Vector3d> wall;
    osg::ref_ptr<osg::Geometry> wallGeom;
    osg::ref_ptr<osg::Vec3Array> wallOSG;
};
   
}
#endif