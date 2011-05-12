set(SOURCES 
        SonarBeamProcessing.cpp
)

set(HEADERS
        SonarBeamProcessing.hpp
)

rock_library(sonar_detectors
    SOURCES ${SOURCES}
    HEADERS ${HEADERS}
    DEPS_PKGCONFIG opencv base-types avalontools
    )

#rock_executable(dummyproject_bin main.cpp
#    DEPS dummyproject)

