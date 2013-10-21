#ifndef WallVisualization_H
#define WallVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/eigen.h>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit3d
{
    
class WallVisualization : public vizkit3d::Vizkit3DPlugin< std::vector<base::Vector3d> >
{
Q_OBJECT
    
public:
    
    WallVisualization();
        
    Q_INVOKABLE void updateWallData( const std::vector<base::Vector3d>& wall_data )
    { return updateData(wall_data); }
    
protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode( osg::Node* node );
    void updateDataIntern ( const std::vector<base::Vector3d>& data );
    
private:
    std::vector<base::Vector3d> wall;
    osg::ref_ptr<osg::Geometry> wallGeom;
    osg::ref_ptr<osg::Vec3Array> wallOSG;
    double wall_length;
};
   
}
#endif