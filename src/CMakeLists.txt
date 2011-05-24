set(SOURCES 
        SonarBeamProcessing.cpp
        WallEstimation.cpp
)

set(HEADERS
        SonarBeamProcessing.hpp
        WallEstimation.hpp
        SonarEstimation.hpp
        SonarDetectorTypes.hpp
)

rock_library(sonar_detectors
    SOURCES ${SOURCES}
    HEADERS ${HEADERS}
    DEPS_PKGCONFIG opencv base-types avalontools
    DEPS_CMAKE Eigen2 #Qt4
    )

#rock_executable(dummyproject_bin main.cpp
#    DEPS dummyproject)

