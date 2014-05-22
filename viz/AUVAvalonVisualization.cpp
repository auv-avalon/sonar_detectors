#include "AUVAvalonVisualization.hpp"
#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>

namespace vizkit3d
{

AUVAvalonVisualization::AUVAvalonVisualization()
{
    VizPluginRubyMethod(AUVAvalonVisualization, bool, showDesiredModelPosition)
    VizPluginRubyMethod(AUVAvalonVisualization, bool, desiredModelPositionRelZ)
    VizPluginRubyMethod(AUVAvalonVisualization, bool, desiredModelPositionRelHeading)
    rbs.invalidate();
    showDesiredModel = false;
    desiredModel_rel_z = false;
    desiredModel_rel_heading = true;
    newPositionCommand = false;
}

osg::ref_ptr< osg::Node > AUVAvalonVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    avalonModelPos = new osg::PositionAttitudeTransform();
    osg::ref_ptr<osg::PositionAttitudeTransform> avalonPosAdjustment = new osg::PositionAttitudeTransform();
    mainNode->addChild(avalonModelPos);
    avalonModelPos->addChild(avalonPosAdjustment);
    
    // get avalon model
    char* osgPath = getenv("OSG_FILE_PATH");
    if (osgPath) 
    {
        std::string filePath(osgPath);
        filePath += "/avalon.osg";
        avalonModel = osgDB::readNodeFile(filePath);
    }
    if (avalonModel.get() == 0)
    {
        avalonModel = printPrimitivModel();
    }
    
    // adjust position of the avalon model
    avalonPosAdjustment->addChild(avalonModel);
    base::Quaterniond quat = Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::Unit(2)) *
        Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::Unit(0)) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::Unit(1));
    osg::Quat orientation = osg::Quat(quat.x(), quat.y(), quat.z(), quat.w());
    avalonPosAdjustment->setAttitude(orientation);
    avalonPosAdjustment->setPosition(osg::Vec3d(0.7,0,0));
    
    // print desired avalon model position
    desiredModelPos = new osg::PositionAttitudeTransform();
    if(showDesiredModel)
    {
        osg::ref_ptr<osg::PositionAttitudeTransform> desiredPosAdjustment = new osg::PositionAttitudeTransform();
        avalonModelPos->addChild(desiredModelPos);
        desiredModelPos->addChild(desiredPosAdjustment);
        
        osg::ref_ptr<osg::Node> desiredModel = printPrimitivModel(0.3);
        desiredPosAdjustment->addChild(desiredModel);
        
        desiredPosAdjustment->setAttitude(orientation);
        desiredPosAdjustment->setPosition(osg::Vec3d(0.7,0,0));
    }
    
    return mainNode;
}

/**
 * This will print a primitive model if the common avalon model wasn't found.
 * 
 * @return primitiv avalon model
 */
osg::ref_ptr< osg::Node > AUVAvalonVisualization::printPrimitivModel(float alpha)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    float height = 1.4;
    float radius = 0.1;
    osg::Vec4 color_cylinder(0.78f, 0.59f, 0.25f, alpha);
    osg::Vec4 color_window(0.70f, 0.71f, 0.87f, alpha);
    osg::ref_ptr<osg::ShapeDrawable> shape1 = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3f(0.0,0.0,height/2.0), radius, height));
    shape1->setColor(color_cylinder);
    osg::ref_ptr<osg::ShapeDrawable> shape2 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(0.0,0.0,0.0), radius));
    shape2->setColor(color_window);
    geode->addDrawable(shape1);
    geode->addDrawable(shape2);
    return geode;
}

void AUVAvalonVisualization::showDesiredModelPosition(bool b)
{
    showDesiredModel = b;
}

void AUVAvalonVisualization::desiredModelPositionRelZ(bool b)
{
    desiredModel_rel_z = b;
}

void AUVAvalonVisualization::desiredModelPositionRelHeading(bool b)
{
    desiredModel_rel_heading = b;
}

double AUVAvalonVisualization::getCurrentDepth()
{
    return rbs.position.z();
}

/**
 * Updates the class with the new body state of type RigidBodyState.
 * 
 * @param data new body state data
 */
void AUVAvalonVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    rbs = data;
    emit propertyChanged("current_depth");
}

/**
 * Updates the class with a new position command.
 * 
 * @param data new position command data
 */
void AUVAvalonVisualization::updateDataIntern(const base::AUVPositionCommand& data)
{
    positionCommand = data;
    newPositionCommand = true;
}

void AUVAvalonVisualization::updateMainNode(osg::Node* node)
{
    //pos.set(rbs.position.x(), rbs.position.y(), rbs.position.z());
    //avalonModelPos->setPosition(pos);
    orientation.set(rbs.orientation.x(), rbs.orientation.y(), rbs.orientation.z(), rbs.orientation.w());
    avalonModelPos->setAttitude(orientation);
    
    if(showDesiredModel && newPositionCommand)
    {
        newPositionCommand = false;
        osg::Vec3d desiredPos(positionCommand.x, positionCommand.y, 
                       (desiredModel_rel_z) ? positionCommand.z : positionCommand.z - rbs.position.z());
        desiredModelPos->setPosition(desiredPos);
        osg::Quat desiredOri(rbs.orientation.x(), rbs.orientation.y(), rbs.orientation.z(), rbs.orientation.w());
        osg::Vec3d top = desiredOri * osg::Vec3d(0,0,1);
        if(desiredModel_rel_heading)
        {
            desiredOri.makeRotate(positionCommand.heading, top.x(), top.y(), top.z());
        }
        else
        {
            double angle;
            desiredOri.getRotate(angle, top.x(), top.y(), top.z());
            desiredOri.makeRotate(positionCommand.heading - angle, top.x(), top.y(), top.z());
        }
        desiredModelPos->setAttitude(desiredOri);
    }
}

}