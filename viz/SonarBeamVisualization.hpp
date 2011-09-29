#ifndef SonarBeamVisualization_H
#define SonarBeamVisualization_H

#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/eigen.h>
#include <base/samples/sonar_beam.h>
#include <base/samples/rigid_body_state.h>
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <sonar_detectors/SonarMap.hpp>

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
class SonarBeamVisualization : public vizkit::Vizkit3DPlugin< base::samples::SonarBeam >,
                               public vizkit::VizPluginAddType< base::samples::RigidBodyState >
{    
    public:
        SonarBeamVisualization();
        const std::string getPluginName() const;
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::SonarBeam& data );
        void updateDataIntern ( const base::samples::RigidBodyState& data );
        
    private:
        base::samples::RigidBodyState bodyState;
        bool newSonarScan;
        double currentAngle;
        avalon::SonarMap< std::vector<avalon::obstaclePoint> > sonarMap;
        std::list< std::vector<avalon::obstaclePoint> > *featureList;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Geometry> beamGeom;
        osg::ref_ptr<osg::Vec3Array> beamPos;
        osg::ref_ptr<osg::DrawArrays> beamDrawArray;
        osg::ref_ptr<osg::Vec4Array> color;
        std::map<uint8_t, osg::Vec4> colorMap;
};

}
#endif
