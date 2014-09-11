#ifndef SONAR_DETECTORS_STRUCTURE_ESTIMATION_HPP_
#define SONAR_DETECTORS_STRUCTURE_ESTIMATION_HPP_

#include <sonar_detectors/SonarEstimation.hpp>
#include <sonar_detectors/SonarMap.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <boost/shared_ptr.hpp>

#include <mtk/types/vect.hpp>
#include <ukfom/ukf.hpp>
#include <ukfom/mtkwrap.hpp>

namespace sonar_detectors
{

/** Measurement model for structure position
 */
template <typename StructType>
StructType measurementModel (const StructType &state)
{
    return state;
}

/** Process model for structure position
 */
template <typename StructType>
StructType processModel (const StructType &state)
{    
    return state;
}

class StructureEstimation : public SonarEstimation
{
public:
    StructureEstimation();
    
    void resetCandidateCounter() {candidate_update_count = 0;}

    void setAUVInOdometry(const Eigen::Affine3d &auvInOdometry) {this->auvInOdometry = auvInOdometry;}
    void setProcessNoiseCovariance(const Eigen::Matrix2d& noise_cov) {process_noise_cov = noise_cov;}
    void setMinValidCadidates(unsigned min_candidates) {this->min_candidates = min_candidates;}
    void setExpectedStructureDiameter(double diameter) {this->structure_diameter = diameter;}
    void setAngularResolutionSonar(double angular_resolution) {this->angular_resolution = angular_resolution;}
    void setMaxMahalanobisDistance(double max_mahalanobis_distance) {this->max_mahalanobis_distance = max_mahalanobis_distance;}
    void setDBScanEpsilon(double dbscan_epsilon) {this->dbscan_epsilon = dbscan_epsilon;}
    
    base::samples::RigidBodyState getStructurePose();
    base::samples::RigidBodyState getStructureConfidence();
    std::vector<base::Vector3d> getClusterPoints() {return cluster_points_debug;}
    unsigned getCandidateUpdateCount() {return candidate_update_count;}
    double getMagnitude() {return mag;}
    std::vector<base::Vector3d> getAUVToStructure() {return auv_to_structure_line;}
    double getDistanceToStructure() {return distance_to_structure;}
    bool isStructureStable();
    
protected:
    typedef MTK::vect<2, double> StructState;
    typedef ukfom::mtkwrap<StructState> WStructState;
    typedef ukfom::ukf<WStructState> UKF;
    typedef UKF::cov Covariance;
    
    virtual void updateFeatureIntern(const base::samples::LaserScan& feature, const Eigen::Affine3d &featureInOdometry);
    virtual void predictionStep(const double delta);
    virtual void correctionStep(const base::Vector3d &position, const Eigen::Matrix3d &cov);
    virtual void initFilter(const base::Vector3d &position, const Eigen::Matrix3d &cov);
    
protected:
    struct StructureFeature
    {
	base::Vector3d point;
	double weight;
    };
    
    boost::shared_ptr<UKF> ukf;
    std::vector<StructureFeature> features;
    double dbscan_epsilon;
    double angular_resolution;
    unsigned min_feature_count;
    Eigen::Affine3d auvInOdometry;
    base::samples::RigidBodyState estimated_structure;
    std::vector<base::Vector3d> cluster_points_debug;
    Covariance process_noise_cov;
    base::Time last_feature_time;
    double max_mahalanobis_distance;
    unsigned candidate_update_count;
    double max_sigma;
    double structure_diameter;
    double mag;
    std::vector<base::Vector3d> auv_to_structure_line;
    double distance_to_structure;
    base::Vector2d cluster2cov;
    unsigned min_candidates;
};

}

#endif