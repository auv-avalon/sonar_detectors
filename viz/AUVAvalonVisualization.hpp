#ifndef AUVAvalonVisualization_H
#define AUVAvalonVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/rigid_body_state.h>
#include <base/motion_command.h>
#include <QObject>

#include <osg/Node>
#include <osg/PositionAttitudeTransform>

namespace vizkit3d
{
    
class AUVAvalonVisualization : public vizkit3d::Vizkit3DPlugin<base::samples::RigidBodyState>,
                               public vizkit3d::VizPluginAddType<base::AUVPositionCommand>
{
    Q_OBJECT
    Q_PROPERTY(double current_depth READ getCurrentDepth)
    
public:
    AUVAvalonVisualization();
    
    Q_INVOKABLE void updateRigidBodyState( const base::samples::RigidBodyState& state )
    { return updateData(state); }
    Q_INVOKABLE void updateDesiredPosition( const base::AUVPositionCommand& command )
    { return updateData(command); }
    
protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode( osg::Node* node );
    void updateDataIntern ( const base::samples::RigidBodyState& data );
    void updateDataIntern ( const base::AUVPositionCommand& data );
    void showDesiredModelPosition(bool b);
    void desiredModelPositionRelZ(bool b);
    void desiredModelPositionRelHeading(bool b);
    double getCurrentDepth();
    
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