set(SOURCES 
        SonarBeamProcessing.cpp
        WallEstimation.cpp
        DistanceEstimation.cpp
        SonarDetectorMath.cpp
        FeatureExtraction.cpp
)

set(HEADERS
        SonarBeamProcessing.hpp
        WallEstimation.hpp
        SonarEstimation.hpp
        SonarDetectorTypes.hpp
        DistanceEstimation.hpp
        SonarDetectorMath.hpp
        SonarMap.hpp
        FeatureExtraction.hpp
)

rock_library(sonar_detectors
    SOURCES ${SOURCES}
    HEADERS ${HEADERS}
    DEPS_PKGCONFIG opencv base-types avalontools eigen3
    )

#rock_executable(dummyproject_bin main.cpp
#    DEPS dummyproject)

