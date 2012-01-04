set(SOURCES 
        WallEstimation.cpp
        DistanceEstimation.cpp
        SonarDetectorMath.cpp
        FeatureExtraction.cpp
        SonarEnvironmentModel.cpp
        PointClustering.cpp
)

set(HEADERS
        WallEstimation.hpp
        SonarEstimation.hpp
        SonarDetectorTypes.hpp
        DistanceEstimation.hpp
        SonarDetectorMath.hpp
        SonarMap.hpp
        FeatureExtraction.hpp
        SonarEnvironmentModel.hpp
        PointClustering.hpp
)

rock_library(sonar_detectors
    SOURCES ${SOURCES}
    HEADERS ${HEADERS}
    DEPS_PKGCONFIG opencv base-types avalontools eigen3 dsp_acoustics machine_learning
    )

#rock_executable(dummyproject_bin main.cpp
#    DEPS dummyproject)

