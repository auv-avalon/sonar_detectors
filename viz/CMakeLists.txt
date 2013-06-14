rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)
    
rock_vizkit_plugin(sonar-viz
    MOC SonarBeamVisualization.cpp SonarFeatureVisualization.cpp SonarDepthVisualization.cpp WallVisualization.cpp AUVAvalonVisualization.cpp PluginLoader.cpp
    DEPS sonar_detectors
    DEPS_PKGCONFIG vizkit-base-viz
    HEADERS SonarBeamVisualization.hpp SonarFeatureVisualization.hpp SonarDepthVisualization.hpp WallVisualization.hpp AUVAvalonVisualization.hpp
    )

# install osg file
install(FILES
    ../resources/avalon.osg
    DESTINATION share/vizkit
    )
