Vizkit::UiLoader.register_3d_plugin('AUVAvalonVisualization', 'sonar', 'AUVAvalonVisualization')
Vizkit::UiLoader.register_3d_plugin_for('AUVAvalonVisualization', "/base/samples/RigidBodyState", :updateRigidBodyState )

Vizkit::UiLoader.register_3d_plugin('SonarBeamVisualization', 'sonar', 'SonarBeamVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarBeamVisualization', "/base/samples/SonarBeam", :updateSonarBeam )

Vizkit::UiLoader.register_3d_plugin('SonarFeatureVisualization', 'sonar', 'SonarFeatureVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarFeatureVisualization', "/base/samples/Pointcloud", :updatePointCloud )

Vizkit::UiLoader.register_3d_plugin('SonarDepthVisualization', 'sonar', 'SonarDepthVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarDepthVisualization', "/sonar_detectors/SonarDepthMap", :updatePointCloud )

Vizkit::UiLoader.register_3d_plugin('WallVisualization', 'sonar', 'WallVisualization')
Vizkit::UiLoader.register_3d_plugin_for('WallVisualization', " /std/vector</base/Vector3d>", :updateWallData )
