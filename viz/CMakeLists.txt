rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)
    
rock_vizkit_plugin(sonar-viz
    MOC SonarBeamVisualization.cpp SonarFeatureVisualization.cpp SonarDepthMapVisualization.cpp WallVisualization.cpp  AUVAvalonVisualization.cpp SonarObstaclesVisualization.cpp PluginLoader.cpp
    DEPS sonar_detectors
    DEPS_PKGCONFIG vizkit3d 
    HEADERS SonarBeamVisualization.hpp SonarFeatureVisualization.hpp SonarDepthMapVisualization.hpp WallVisualization.hpp AUVAvalonVisualization.hpp SonarObstaclesVisualization.hpp
    )

# install osg file
install(FILES
    ../resources/avalon.osg
    DESTINATION share/vizkit
    )
