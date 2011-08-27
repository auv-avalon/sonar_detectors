#include "SonarBeamVisualization.hpp"
#include <osgDB/ReadFile>
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <avalonmath.h>

namespace vizkit
{

SonarBeamVisualization::SonarBeamVisualization()
{
    // initialize values
    newSonarValue = false;
    newRigidBodyState = false;
    wallEstimation = 0;
    processing = new avalon::SonarBeamProcessing(avalon::globalMaximum, avalon::persistNewScans);
    processing->setBeamThreshold(1, 100);
    processing->enableBeamThreshold(true);
    processing->setMinResponseValue(10);
    settings.startAngle = 0;
    settings.endAngle = 2.0 * M_PI;
    settings.segMode = avalon::forEachBeam;
    processing->addSonarEstimation(this);
    
    VizPluginRubyAdapter(SonarBeamVisualization, base::samples::SonarScan, SonarScan)
    VizPluginRubyAdapter(SonarBeamVisualization, base::samples::RigidBodyState, RigidBodyState)
}

/**
 * Creates the main node and attachs the avalon model.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > SonarBeamVisualization::createMainNode()
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
    base::Quaterniond quat = Avalonmath::eulerToQuaternion(-M_PI / 2.0, 0.0, M_PI / 2.0);
    osg::Quat orientation = osg::Quat(quat.x(), quat.y(), quat.z(), quat.w());
    avalonPosAdjustment->setAttitude(orientation);
    avalonPosAdjustment->setPosition(osg::Vec3d(0.7,0,0));
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    osg::Vec4Array* color = new osg::Vec4Array;
    color->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());
    
    // set up wall geometry
    wallGeom = new osg::Geometry;
    wallOSG = new osg::Vec3Array;
    wallGeom->setVertexArray(wallOSG);
    osg::Vec4Array* wallcolor = new osg::Vec4Array;
    wallcolor->push_back(osg::Vec4(0.0f, 0.8f, 0.8f, 1.0f));
    wallGeom->setColorArray(wallcolor);
    wallGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    //draw virtual point
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3d(0,0,0), 0.05);
    osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sp.get());
    sd->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    osg::ref_ptr<osg::Geode> spGeode = new osg::Geode();
    spGeode->addDrawable(sd.get());
    virtualPoint = new osg::PositionAttitudeTransform();
    virtualPoint->addChild(spGeode);

    geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    geode->addDrawable(wallGeom.get());
    
    mainNode->addChild(geode);
    mainNode->addChild(virtualPoint);
    
    return mainNode;
}

/**
 * This will print a primitive model if the common avalon model wasn't found.
 * 
 * @return primitiv avalon model
 */
osg::ref_ptr< osg::Node > SonarBeamVisualization::printPrimitivModel()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    float height = 1.4;
    float radius = 0.1;
    osg::Vec4 color_cylinder(0.78f, 0.59f, 0.25f, 1.0f);
    osg::Vec4 color_window(0.70f, 0.71f, 0.87f, 1.0f);
    osg::ref_ptr<osg::ShapeDrawable> shape1 = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3f(0.0,0.0,height/2.0), radius, height));
    shape1->setColor(color_cylinder);
    osg::ref_ptr<osg::ShapeDrawable> shape2 = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(0.0,0.0,0.0), radius));
    shape2->setColor(color_window);
    geode->addDrawable(shape1);
    geode->addDrawable(shape2);
    return geode;
}

void SonarBeamVisualization::createDockWidgets()
{
    
}

/**
 * @return plugin name
 */
const std::string SonarBeamVisualization::getPluginName() const
{
    return "SonarBeamVisualization";
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarBeamVisualization::updateDataIntern(const base::samples::SonarScan& data)
{
    sonar = data;
    newSonarValue = true;
}

/**
 * Updates the class with the new body state of type RigidBodyState.
 * 
 * @param data new body state data
 */
void SonarBeamVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    rigidBodyState = data;
    newRigidBodyState = true;
}

/**
 * Callback method to update the sonar beam point cloud.
 * 
 * @param segment point cloud
 */
void SonarBeamVisualization::updateSegment(const avalon::scanSegment& segment)
{
    pointsOSG->clear();
    for(std::list<avalon::obstaclePoint>::const_iterator lit = segment.pointCloud.begin(); lit != segment.pointCloud.end(); lit++)
    {
        base::Position pos = lit->position;
        osg::Vec3d vec(pos.x(), pos.y(), pos.z());
        pointsOSG->push_back(vec);
        pointsOSG->push_back(vec + osg::Vec3d(0,0,lit->value)/50.0);
    }
}

/**
 * Adds a wall estimator. This results in an estimation and drawing 
 * of a wall in the point cloud.
 * 
 * @param wallEstimation wall estimator
 */
void SonarBeamVisualization::addWallEstimation(avalon::WallEstimation* wallEstimation)
{
    processing->addSonarEstimation(wallEstimation);
    this->wallEstimation = wallEstimation;
}

/**
 * Removes the wall estimator.
 * 
 * @param wallEstimation wall estimator
 */
void SonarBeamVisualization::removeWallEstimation(avalon::WallEstimation* wallEstimation)
{
    if (this->wallEstimation == wallEstimation)
        this->wallEstimation = 0;
}

/**
 * @return the sonar beam processing class
 */
avalon::SonarBeamProcessing* SonarBeamVisualization::getSonarDetector() const
{
    return processing;
}

/**
 * Main callback method of osg to update all drawings.
 * The method updates the position if there is a new one and
 * adds new sonar beams if there are new ones. If a wall 
 * estimatior is connected the position of the wall will 
 * be updated too.
 * 
 * @param node osg main node
 */
void SonarBeamVisualization::updateMainNode(osg::Node* node)
{
    // update body state
    if (newRigidBodyState)
    {
        newRigidBodyState = false;
        base::samples::RigidBodyState& rbs(rigidBodyState);

        /*
        if(rbs.hasValidPosition()) {
            processing->updatePosition(rbs.position);
            pos.set(rbs.position.x(), rbs.position.y(), rbs.position.z());
            avalonModelPos->setPosition(pos);
        }

        if(rbs.hasValidOrientation()) {
            if(isnan(rbs.orientation.x()) || isnan(rbs.orientation.y()) || isnan(rbs.orientation.z()) || 
            isnan(rbs.orientation.w()) || isinf(rbs.orientation.x()) || isinf(rbs.orientation.y()) || 
            isinf(rbs.orientation.z()) || isinf(rbs.orientation.w()))
            {
                std::cout << "BAD ORIENATION !!!!!" <<std::endl;
            }
            else    
            {
                processing->updateOrientation(rbs.orientation);
                orientation.set(rbs.orientation.x(), rbs.orientation.y(), rbs.orientation.z(), rbs.orientation.w());
                avalonModelPos->setAttitude(orientation);
            }
        }
        */
        processing->updatePosition(rbs.position);
        pos.set(rbs.position.x(), rbs.position.y(), rbs.position.z());
        avalonModelPos->setPosition(pos);
        processing->updateOrientation(rbs.orientation);
        orientation.set(rbs.orientation.x(), rbs.orientation.y(), rbs.orientation.z(), rbs.orientation.w());
        avalonModelPos->setAttitude(orientation);
    }
    // update sonar data
    if(newSonarValue)
    {
        newSonarValue = false;
        processing->updateSonarData(sonar);
        wallOSG->clear();
        
        // update wall position
        if (wallEstimation)
        {
            const std::vector< std::pair<base::Vector3d, base::Vector3d> > walls = wallEstimation->getWalls();
            const base::Vector3d virtualpoint =  wallEstimation->getVirtualPoint();
            if(walls.size())
            {
                for(std::vector< std::pair<base::Position, base::Position> >::const_iterator wit = walls.begin(); wit != walls.end(); wit++)
                {
                    base::Position pos1 = wit->first - wit->second * 5;
                    base::Position pos2 = wit->first + wit->second * 5;
                    osg::Vec3d vec1(pos1.x(), pos1.y(), pos1.z());
                    osg::Vec3d vec2(pos2.x(), pos2.y(), pos2.z());
                    wallOSG->push_back(vec1 + osg::Vec3d(0,0,1));
                    wallOSG->push_back(vec1 + osg::Vec3d(0,0,-1));
                    wallOSG->push_back(vec2 + osg::Vec3d(0,0,1));
                    wallOSG->push_back(vec2 + osg::Vec3d(0,0,-1));
                }
            }
            virtualPoint->setPosition(osg::Vec3d(virtualpoint.x(), virtualpoint.y(), virtualpoint.z()));
            
            wallGeom->removePrimitiveSet(0, wallGeom->getNumPrimitiveSets());
            if(wallOSG->size() >= 4) 
            {
                wallGeom->setVertexArray(wallOSG);
                for(int i = 0; i < wallOSG->size() - 3; i += 4)
                {
                    osg::ref_ptr<osg::DrawElementsUInt> drawWall = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
                    drawWall->push_back(i);
                    drawWall->push_back(i+1);
                    drawWall->push_back(i+3);
                    drawWall->push_back(i+2);
                    wallGeom->addPrimitiveSet(drawWall);
                }
            }
        }
        else
        {
            virtualPoint->setPosition(osg::Vec3d(0, 0, 0));
            wallGeom->removePrimitiveSet(0, wallGeom->getNumPrimitiveSets());
        }
        
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
    }
}

    
}
