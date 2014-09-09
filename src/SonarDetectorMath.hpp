#ifndef SONAR_DETECTOR_MATH_HPP_
#define SONAR_DETECTOR_MATH_HPP_

#include <vector>
#include <base/eigen.h>

namespace base
{
    class Angle;
}

namespace sonar_detectors
{  
    const static double right_angle_variance = M_PI / 18.0; // ~10°
    
    double wallRansac(const std::vector< base::Vector3d >& pointCloud, unsigned iterations, double threshold, double fit_rate, std::vector< std::pair< base::Vector3d, base::Vector3d > >& best_models);
    double ransac(const std::vector< base::Vector3d >& pointCloud, unsigned iterations, double threshold, std::vector< base::Vector3d >& outlier_best_model, std::pair< base::Vector3d, base::Vector3d >& best_model);
    double lineFitEvaluation(const std::vector< base::Vector3d >& pointCloud, const std::pair< base::Vector3d, base::Vector3d >& line, const double threshold, std::vector< base::Vector3d >& inlier, std::vector< base::Vector3d >& outlier);
    void computeLine(const std::vector<base::Vector3d>& pointCloud, std::pair<base::Vector3d, base::Vector3d>& line);
    double computeDistance(const base::Vector3d& vec1, const base::Vector3d& vec2);
    double computeDistance(const std::pair<base::Vector3d, base::Vector3d>& line, const base::Vector3d& point);
    double computeAngle(const std::pair< base::Vector3d, base::Vector3d >& line1, const std::pair< base::Vector3d, base::Vector3d >& line2);
    double computeAngle(const base::Vector3d& vec1, const base::Vector3d& vec2);
    double computeRotation(const base::Vector2d& vec);
    double length(const base::Vector3d& vec);
    double length(const base::Vector2d& vec);
    base::Vector3d computIntersection(const std::pair<base::Vector3d, base::Vector3d>& line, const base::Vector3d& point);
    bool isInAngularRange(const base::Angle& angle, const base::Angle& left_limit, const base::Angle& right_limit);
}

#endif