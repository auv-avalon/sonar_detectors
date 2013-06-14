set(SOURCES 
        RansacWallEstimation.cpp
        CenterWallEstimation.cpp
        DistanceEstimation.cpp
        SonarDetectorMath.cpp
        FeatureExtraction.cpp
        SonarEnvironmentModel.cpp
        PointClustering.cpp
        MWallEstimation.cpp
)

set(HEADERS
        SonarEstimation.hpp
        SonarDetectorTypes.hpp
        RansacWallEstimation.hpp
        CenterWallEstimation.hpp
        DistanceEstimation.hpp
        SonarDetectorMath.hpp
        SonarMap.hpp
        FeatureExtraction.hpp
        SonarEnvironmentModel.hpp
        PointClustering.hpp
        SonarWallMap.hpp
        MWallEstimation.hpp
        SonarDepthMap.hpp
)

rock_library(sonar_detectors
    SOURCES ${SOURCES}
    HEADERS ${HEADERS}
    DEPS_PKGCONFIG opencv base-types avalontools eigen3 dsp_acoustics machine_learning
    )
