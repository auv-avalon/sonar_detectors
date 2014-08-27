#ifndef SonarDetectorVisualization_H
#define SonarDetectorVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <sonar_detectors/SonarDetectorTypes.hpp>

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
class SonarDetectorVisualization : public vizkit3d::Vizkit3DPlugin< sonar_detectors::SonarFeatures >,
                                  public vizkit3d::VizPluginAddType< std::vector< base::Vector3d > >
{    
    Q_OBJECT
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    
    public:
        SonarDetectorVisualization();
        
        Q_INVOKABLE void updateSonarFeatures( const sonar_detectors::SonarFeatures& sample )
        { return updateData(sample); }
        Q_INVOKABLE void updateChannelData( const std::vector< base::Vector3d >& channel_data )
        { return updateData(channel_data); }
        
    public slots:
        QColor getDefaultFeatureColor();
        void setDefaultFeatureColor(QColor color);
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const sonar_detectors::SonarFeatures& data );
        void updateDataIntern ( const std::vector< base::Vector3d >& data );
        
    private:

        sonar_detectors::SonarFeatures features;
        osg::Vec4f default_feature_color;
        std::vector< base::Vector3d > channelInfos;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        bool newPoints;
};

}
#endif
