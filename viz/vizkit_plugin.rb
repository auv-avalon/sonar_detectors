Vizkit::UiLoader.register_3d_plugin('AUVAvalonVisualization', 'sonar', 'AUVAvalonVisualization')
Vizkit::UiLoader.register_3d_plugin_for('AUVAvalonVisualization', "/base/samples/RigidBodyState", :updateRigidBodyState )

Vizkit::UiLoader.register_3d_plugin('AvalonSonarBeamVisualization', 'sonar', 'AvalonSonarBeamVisualization')
Vizkit::UiLoader.register_3d_plugin_for('AvalonSonarBeamVisualization', "/base/samples/SonarBeam", :updateSonarBeam )

Vizkit::UiLoader.register_3d_plugin('SonarFeatureVisualization', 'sonar', 'SonarFeatureVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarFeatureVisualization', "/base/samples/Pointcloud", :updatePointCloud )

Vizkit::UiLoader.register_3d_plugin('SonarObstaclesVisualization', 'sonar', 'SonarObstaclesVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarObstaclesVisualization', "sonar_detectors/ObstacleFeatures", :updateObstacles )
Vizkit::UiLoader.register_3d_plugin_for('SonarObstaclesVisualization', "base/samples/RigidBOdyState", :updatePose )

Vizkit::UiLoader.register_3d_plugin('SonarDepthMapVisualization', 'sonar', 'SonarDepthMapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarDepthMapVisualization', "/base/samples/Pointcloud", :updatePointCloud )

Vizkit::UiLoader.register_3d_plugin('SonarDetectorVisualization', 'sonar', 'SonarDetectorVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarDetectorVisualization', "/sonar_detectors/SonarFeatures", :updateSonarFeatures )

Vizkit::UiLoader.register_3d_plugin('WallVisualization', 'sonar', 'WallVisualization')
Vizkit::UiLoader.register_3d_plugin_for('WallVisualization', " /std/vector</base/Vector3d>", :updateWallData )
