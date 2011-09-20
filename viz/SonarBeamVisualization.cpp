#include "SonarBeamVisualization.hpp"
#include <osg/Geode>
#include <sonar_detectors/SonarBeamProcessing.hpp>

namespace vizkit
{

SonarBeamVisualization::SonarBeamVisualization()
{
    VizPluginRubyAdapter(SonarBeamVisualization, base::samples::SonarScan, SonarScan);
    VizPluginRubyAdapter(SonarBeamVisualization, base::samples::RigidBodyState, BodyState);
    bodyState.invalidate();
    newSonarScan = false;
    currentAngle = 0;
    featureList = sonarMap.getFeatureListPtr();
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > SonarBeamVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
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
    
    // set up beam line
    beamGeom = new osg::Geometry;
    beamPos = new osg::Vec3Array;
    beamPos->push_back(osg::Vec3d(0.0,0.0,0.0));
    beamPos->push_back(osg::Vec3d(0.0,0.0,0.0));
    beamGeom->setVertexArray(beamPos);
    osg::Vec4Array* beam_color = new osg::Vec4Array;
    beam_color->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
    beamGeom->setColorArray(beam_color);
    beamGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
    beamGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    beamDrawArray = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, beamPos->size() );
    beamGeom->addPrimitiveSet(beamDrawArray.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    geode->addDrawable(beamGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;
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
    std::vector<avalon::obstaclePoint> features;
    for(int i = 0; i < data.scanData.size(); i++)
    {
        if(data.scanData[i] > 0)
            features.push_back(avalon::SonarBeamProcessing::computeObstaclePoint(i, data, bodyState.orientation));
    }
    sonarMap.addFeature(features, data.angle, data.time);
    currentAngle = data.angle;
    newSonarScan = true;
}

/**
 * Updates the class with the new sonar data of type SonarScan.
 * 
 * @param data new sonar data
 */
void SonarBeamVisualization::updateDataIntern(const base::samples::RigidBodyState& data)
{
    bodyState = data;
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
    if (newSonarScan)
    {
        newSonarScan = false;
        // draw sonar data
        pointsOSG->clear();
        for(std::list< std::vector<avalon::obstaclePoint> >::const_iterator l_it = featureList->begin(); l_it != featureList->end(); l_it++)
        {
            for(std::vector<avalon::obstaclePoint>::const_iterator v_it = l_it->begin(); v_it != l_it->end(); v_it++)
            {
                osg::Vec3d vec(v_it->position.x(), v_it->position.y(), v_it->position.z());
                pointsOSG->push_back(vec);
                pointsOSG->push_back(vec + osg::Vec3d(0,0,v_it->value)/51.0);
            }
            
        }
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        
        // draw current beam position
        Eigen::Vector3d beam_range(-24,0,0);
        Eigen::Vector3d topPoint(0,0,1);
        beam_range = bodyState.orientation * beam_range;
        topPoint = bodyState.orientation * topPoint;
        Eigen::AngleAxisd rotate(-currentAngle,topPoint);
        beam_range = rotate * beam_range;
        beamPos->begin()->x() = beam_range.x();
        beamPos->begin()->y() = beam_range.y();
        beamPos->begin()->z() = beam_range.z();
        beamGeom->setVertexArray(beamPos);
    }
}

VizkitQtPlugin(SonarBeamVisualization)
}