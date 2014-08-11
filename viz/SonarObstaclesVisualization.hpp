#ifndef SONARDETECTORS_SONAROBSTACLESVISUALIZATION_HPP
#define SONARDETECTORS_SONAROBSTACLESVISUALIZATION_HPP

#include <base/samples/LaserScan.hpp>
#include <sonar_detectors/SonarDetectorTypes.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>

/**
 * Vizkit plugin for obstacle features
 * This is an adaption of the rock laserscan-visualization
 */


namespace osg {
    class Geometry;
}

namespace vizkit3d {

class SonarObstaclesVisualization : public Vizkit3DPlugin<sonar_detectors::ObstacleFeatures>, public VizPluginAddType<base::samples::RigidBodyState>
{
    Q_OBJECT

    Q_PROPERTY(bool YForward READ isYForwardModeEnabled WRITE setYForwardMode)
    Q_PROPERTY(bool Colorize READ isColorizeEnabled WRITE setColorize)
    Q_PROPERTY(double ColorizeInterval READ getColorizeInterval WRITE setColorizeInterval)
    Q_PROPERTY(bool ShowPolygon READ isShowPolygonEnabled WRITE setShowPolygon)
    bool mYForward;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SonarObstaclesVisualization();
    ~SonarObstaclesVisualization();
    virtual void updateDataIntern(const sonar_detectors::ObstacleFeatures& data);
    virtual void updateDataIntern(const base::samples::RigidBodyState& data);
    virtual void updateMainNode(osg::Node* node);
    virtual osg::ref_ptr< osg::Node > createMainNode();

    Q_INVOKABLE void updateData(const sonar_detectors::ObstacleFeatures& data)
    { Vizkit3DPlugin<sonar_detectors::ObstacleFeatures>::updateData(data); }
    Q_INVOKABLE void updateObstacles(const sonar_detectors::ObstacleFeatures& data)
    { updateData(data); }
    Q_INVOKABLE void updateData(const base::samples::RigidBodyState& data)
    { Vizkit3DPlugin<sonar_detectors::ObstacleFeatures>::updateData(data); }
    Q_INVOKABLE void updatePose(const base::samples::RigidBodyState& data)
    { updateData(data); }

public slots:
    bool isYForwardModeEnabled() const;
    void setYForwardMode(bool enabled);
    void setColorize(bool value);
    bool isColorizeEnabled()const;

    //interval in meter 0 = black , interval = white 
    void setColorizeInterval(double value);
    double getColorizeInterval()const;

    void setShowPolygon(bool value);
    bool isShowPolygonEnabled()const;

protected:
    osg::ref_ptr<osg::Node> cloneCurrentViz();

private:
    sonar_detectors::ObstacleFeatures scan;
    Eigen::Vector3d scanPosition;
    Eigen::Quaterniond scanOrientation;
    osg::ref_ptr< osg::PositionAttitudeTransform > transformNode;
    osg::ref_ptr<osg::Geode> scanNode;
    osg::ref_ptr<osg::Geometry> scanGeom;

    bool colorize;
    bool show_polygon;
    double colorize_interval;   // 1/distance 
};

}

#endif // LASERSCANVISUALIZATION_H
