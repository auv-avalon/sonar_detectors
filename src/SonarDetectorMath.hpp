#ifndef SONAR_DETECTOR_MATH_HPP_
#define SONAR_DETECTOR_MATH_HPP_

#include <vector>
#include <base/eigen.h>

namespace avalon
{  
    double ransac(const std::vector<base::Vector3d>& pointCloud, int iterations, double threshold, double fit_rate, std::pair<base::Vector3d, base::Vector3d>& best_model);
    void computeModel(const std::vector<base::Vector3d>& pointCloud, std::pair<base::Vector3d, base::Vector3d>& model);
    double computeDistance(const base::Vector3d& vec1, const base::Vector3d& vec2);
    double computeDistance(const std::pair<base::Vector3d, base::Vector3d>& line, const base::Vector3d& point);
    base::Vector3d computIntersection(const std::pair<base::Vector3d, base::Vector3d>& line, const base::Vector3d& point);
}

#endif