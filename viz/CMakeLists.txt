rock_vizkit_plugin(sonarbeam-viz
    SonarBeamVisualization.cpp
    DEPS sonar_detectors
    HEADERS SonarBeamVisualization.hpp)

# install osg file
install(FILES
    ../resources/avalon.osg
    DESTINATION share/vizkit
    )
