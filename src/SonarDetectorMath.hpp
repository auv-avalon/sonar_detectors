#ifndef SONAR_DETECTOR_MATH_HPP_
#define SONAR_DETECTOR_MATH_HPP_

#include <vector>
#include <base/eigen.h>

namespace sonar_detectors
{  
    const static double right_angle_variance = M_PI / 18.0; // ~10°
    
    double wallRansac(const std::vector< base::Vector3d >& pointCloud, int iterations, double threshold, double fit_rate, std::vector< std::pair< base::Vector3d, base::Vector3d > >& best_models);
    double ransac(const std::vector< base::Vector3d >& pointCloud, int iterations, double threshold, std::vector< base::Vector3d >& outlier_best_model, std::pair< base::Vector3d, base::Vector3d >& best_model);
    void computeModel(const std::vector<base::Vector3d>& pointCloud, std::pair<base::Vector3d, base::Vector3d>& model);
    double computeDistance(const base::Vector3d& vec1, const base::Vector3d& vec2);
    double computeDistance(const std::pair<base::Vector3d, base::Vector3d>& line, const base::Vector3d& point);
    double computeAngle(const std::pair< base::Vector3d, base::Vector3d >& line1, const std::pair< base::Vector3d, base::Vector3d >& line2);
    double length(const base::Vector3d& vec);
    base::Vector3d computIntersection(const std::pair<base::Vector3d, base::Vector3d>& line, const base::Vector3d& point);
}

#endif