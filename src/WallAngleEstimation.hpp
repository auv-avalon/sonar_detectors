#ifndef SONAR_DETECTORS_WALL_ANGLE_ESTIMATION_HPP_
#define SONAR_DETECTORS_WALL_ANGLE_ESTIMATION_HPP_

#include <sonar_detectors/SonarEstimation.hpp>
#include <sonar_detectors/SonarMap.hpp>
#include <sonar_detectors/WallAngleEstimationCandidate.hpp>

namespace sonar_detectors
{

class WallAngleEstimation : public SonarEstimation
{
public:
    WallAngleEstimation();
    void setRansacParameters(double ransac_threshold, double ransac_fit_rate);
    void setStabilityParameters(unsigned wall_candidate_count, double max_angle_sigma, double max_distance_sigma);
    bool getWallAngle(base::Angle& angle);
    bool getAngleToWall(base::Angle& angle);
    bool getDistanceToWall(double& distance);
    const std::vector< WallCandidate >& getCandidates() {return wall_candidates;}
    const std::vector<base::Vector3d>& getFeatures() {return debug_feature_cloud;}
    
protected:
    virtual void updateFeatureIntern(const base::samples::LaserScan& feature, const Eigen::Affine3d &featureInOdometry);
    bool estimateLine(const std::vector<base::Vector3d>& point_cloud, std::pair<base::Vector3d, base::Vector3d>& line);
    bool computeMean(WallCandidate& mean);
    
protected:
    sonar_detectors::SonarMap< base::Vector3d > sonarMap;
    std::vector<base::Vector3d> debug_feature_cloud;
    std::vector< WallCandidate > wall_candidates;
    unsigned wall_candidate_count;
    unsigned min_feature_count;
    double ransac_threshold;
    double ransac_fit_rate;
    base::Angle angle_to_wall;
    base::Angle wall_angle;
    double distance_to_wall;
    double max_angle_sigma;
    double max_distance_sigma;
};

}

#endif