#ifndef SonarBeamVisualization_H
#define SonarBeamVisualization_H

#include <iostream>
#include <list>

#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/samples/sonar_scan.h>
#include <base/samples/rigid_body_state.h>
#include <base/eigen.h>
#include <sonar_detectors/SonarEstimation.hpp>
#include <sonar_detectors/WallEstimation.hpp>
#include <sonar_detectors/SonarBeamProcessing.hpp>

#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>

namespace vizkit
{

/**
 * Vizkit plugin to visualize sonar data.
 * 
 * If the class gets updated with a body state the sonar 
 * data is absolute, otherwise relative.
 */
class SonarBeamVisualization : public vizkit::Vizkit3DPlugin<base::samples::SonarScan>,
                               public vizkit::VizPluginAddType<base::samples::RigidBodyState>,
                               public avalon::SonarEstimation
{    
    public:
        SonarBeamVisualization();
        const std::string getPluginName() const;
        void addWallEstimation(avalon::WallEstimation* wallEstimation);
        void removeWallEstimation(avalon::WallEstimation* wallEstimation);
        avalon::SonarBeamProcessing* getSonarDetector() const;
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        virtual void createDockWidgets();
        void updateDataIntern ( const base::samples::SonarScan& data );
        void updateDataIntern ( const base::samples::RigidBodyState& data );
        virtual void updateSegment(const avalon::scanSegment& segment);
        
        
    private:
        osg::ref_ptr< osg::Node > printPrimitivModel();
        
        avalon::SonarBeamProcessing* processing;
        avalon::WallEstimation* wallEstimation;
        base::samples::RigidBodyState rigidBodyState;
        base::samples::SonarScan sonar;
        bool newSonarValue;
        bool newRigidBodyState;
        osg::ref_ptr<osg::PositionAttitudeTransform> avalonModelPos;
        osg::ref_ptr<osg::Node> avalonModel;
        osg::Quat orientation;
        osg::Vec3d pos;
        osg::ref_ptr<osg::Geode> geode;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Geometry> wallGeom;
        osg::ref_ptr<osg::Vec3Array> wallOSG;
        osg::ref_ptr<osg::PositionAttitudeTransform> virtualPoint;
};

VizkitQtPlugin(SonarBeamVisualization)

}
#endif
