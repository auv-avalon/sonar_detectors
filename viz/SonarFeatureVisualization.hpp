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
class SonarFeatureVisualization : public vizkit::Vizkit3DPlugin< base::samples::Pointcloud >
{    
    public:
        SonarFeatureVisualization();
        const std::string getPluginName() const;
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::Pointcloud& data );
        
    private:
        base::samples::Pointcloud pointCloud;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
};

}
#endif
