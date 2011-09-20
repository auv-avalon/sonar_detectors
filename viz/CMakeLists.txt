rock_vizkit_plugin(sonarbeam-viz
    SonarBeamVisualization.cpp
    DEPS sonar_detectors
    HEADERS SonarBeamVisualization.hpp)

rock_vizkit_plugin(auv_avalon-viz
    AUVAvalonVisualization.cpp
    DEPS sonar_detectors
    HEADERS AUVAvalonVisualization.hpp)

# install osg file
install(FILES
    ../resources/avalon.osg
    DESTINATION share/vizkit
    )
