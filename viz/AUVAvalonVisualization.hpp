#ifndef AUVAvalonVisualization_H
#define AUVAvalonVisualization_H

#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/samples/rigid_body_state.h>
#include <base/motion_command.h>

#include <osg/Node>
#include <osg/PositionAttitudeTransform>

namespace vizkit
{
    
class AUVAvalonVisualization : public vizkit::Vizkit3DPlugin<base::samples::RigidBodyState>,
                               public vizkit::VizPluginAddType<base::AUVPositionCommand>
{
public:
    AUVAvalonVisualization();
    
protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode( osg::Node* node );
    void updateDataIntern ( const base::samples::RigidBodyState& data );
    void updateDataIntern ( const base::AUVPositionCommand& data );
    void showDesiredModelPosition(bool b);
    void desiredModelPositionRelZ(bool b);
    void desiredModelPositionRelHeading(bool b);
    
private:
    osg::ref_ptr< osg::Node > printPrimitivModel(float alpha = 1.0);
    
    base::samples::RigidBodyState rbs;
    base::AUVPositionCommand positionCommand;
    osg::ref_ptr<osg::PositionAttitudeTransform> avalonModelPos;
    osg::ref_ptr<osg::PositionAttitudeTransform> desiredModelPos;
    osg::ref_ptr<osg::Node> avalonModel;
    osg::Quat orientation;
    osg::Vec3d pos;
    bool showDesiredModel;
    bool desiredModel_rel_z;
    bool desiredModel_rel_heading;
    bool newPositionCommand;
};
   
}
#endif