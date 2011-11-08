#ifndef SonarFeatureVisualization_H
#define SonarFeatureVisualization_H

#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/samples/pointcloud.h>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit
{

/**
 * Vizkit plugin to visualize sonar data.
 * 
 * If the class gets updated with a body state the sonar 
 * data is absolute, otherwise relative.
 */
class SonarFeatureVisualization : public vizkit::Vizkit3DPlugin< base::samples::Pointcloud >,
                                  public vizkit::VizPluginAddType< std::vector< base::Vector3d > >
{    
    public:
        SonarFeatureVisualization();
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::Pointcloud& data );
        void updateDataIntern ( const std::vector< base::Vector3d >& data );
        
    private:
        base::samples::Pointcloud pointCloud;
        std::vector< base::Vector3d > channelInfos;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        bool newPoints;
};

}
#endif
