rock_vizkit_plugin(sonarbeam-viz
    SonarBeamVisualization.cpp
    DEPS sonar_detectors
    HEADERS SonarBeamVisualization.hpp)

rock_vizkit_plugin(sonarfeature-viz
    SonarFeatureVisualization.cpp
    DEPS sonar_detectors
    HEADERS SonarFeatureVisualization.hpp
    MOC SonarFeatureVisualization.hpp)

rock_vizkit_plugin(wall-viz
    WallVisualization.cpp
    DEPS sonar_detectors
    HEADERS WallVisualization.hpp)

rock_vizkit_plugin(auv_avalon-viz
    AUVAvalonVisualization.cpp
    DEPS sonar_detectors
    HEADERS AUVAvalonVisualization.hpp
    MOC AUVAvalonVisualization.hpp)

# install osg file
install(FILES
    ../resources/avalon.osg
    DESTINATION share/vizkit
    )
