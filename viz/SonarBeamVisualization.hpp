#ifndef SonarBeamVisualization_H
#define SonarBeamVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/sonar_beam.h>
#include <base/samples/rigid_body_state.h>
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <sonar_detectors/SonarMap.hpp>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit3d
{

/**
 * Vizkit plugin to visualize sonar data.
 * 
 * If the class gets updated with a body state the sonar 
 * data is absolute, otherwise relative.
 */
class SonarBeamVisualization : public vizkit3d::Vizkit3DPlugin< base::samples::SonarBeam >,
                               public vizkit3d::VizPluginAddType< base::samples::RigidBodyState >
{    
    Q_OBJECT
    
    public:
        SonarBeamVisualization();
    
        Q_INVOKABLE void updateSonarBeam( const base::samples::SonarBeam& sample )
        { return updateData(sample); }
        Q_INVOKABLE void updateOrientation( const base::samples::RigidBodyState& orientation )
        { return updateData(orientation); }
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const base::samples::SonarBeam& data );
        void updateDataIntern ( const base::samples::RigidBodyState& data );
        
    private:
        base::samples::RigidBodyState bodyState;
        bool newSonarScan;
        double currentAngle;
        sonar_detectors::SonarMap< std::vector<sonar_detectors::obstaclePoint> > sonarMap;
        std::list< std::vector<sonar_detectors::obstaclePoint> > *featureList;
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
