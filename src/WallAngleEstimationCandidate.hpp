#ifndef SONAR_DETECTORS_WALL_ANGLE_ESTIMATION_CANDIDATE_HPP_
#define SONAR_DETECTORS_WALL_ANGLE_ESTIMATION_CANDIDATE_HPP_

#include <base/Angle.hpp>

namespace sonar_detectors
{

struct WallCandidate
{
    base::Angle angle_to_wall;
    base::Angle wall_angle;
    double wall_distance;
};

}

#endif