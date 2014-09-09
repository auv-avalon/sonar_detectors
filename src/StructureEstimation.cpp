#include "StructureEstimation.hpp"
#include <sonar_detectors/SonarDetectorMath.hpp>
#include <sonar_detectors/PointClustering.hpp>

namespace sonar_detectors
{

StructureEstimation::StructureEstimation()
{
    dbscan_epsilon = 10.0;
    angular_resolution = 0.019638; // 1.13°
    min_feature_count = 5;
    auvInOdometry = Eigen::Affine3d::Identity();
    estimated_structure.invalidate();
    process_noise_cov = 0.05 * Covariance::Identity();
    last_feature_time.microseconds = 0;
    ukf.reset();
    max_mahalanobis_distance = 1.0;
    candidate_update_count = 0;
    structure_diameter = 3.0; // the structure is 2x3x1.8m
    max_sigma = structure_diameter * 0.5;
    cluster2cov = base::Vector2d::Zero();
    min_candidates = 3;
}

void StructureEstimation::updateFeatureIntern(const base::samples::LaserScan& feature, const Eigen::Affine3d& featureInOdometry)
{    
    // prediction step
    if(ukf.get() != NULL)
    {
	double delta = (feature.time - last_feature_time).toSeconds();
	if(delta >= 0.0 && delta <= 2.0)
	    predictionStep((feature.time - last_feature_time).toSeconds());
	else
	    ukf.reset();
    }
    last_feature_time = feature.time;
    
    // collect features
    if(inside_estimation_zone)
    {
	std::vector<base::Vector3d> new_feature;
	feature.convertScanToPointCloud(new_feature, featureInOdometry);
	if(new_feature.size() > 0)
	{
	    // add new feature
	    for(unsigned i = 0; i < new_feature.size(); i++)
	    {
		if(base::isnotnan(new_feature[i]))
		{
		    StructureFeature structure_feature;
		    structure_feature.point = new_feature[i];
		    structure_feature.weight = 0.5 + 0.5 * feature.remission[i];
		    features.push_back(structure_feature);
		}
	    }
	}
    }
    
    // check if covariance is good enough
    if(ukf.get() != NULL)
    {
	Eigen::Matrix2d diff = Eigen::Matrix2d::Identity() * std::pow(max_sigma, 2) - ukf->sigma();
	//Eigen::Matrix2d diff = Eigen::Matrix2d::Identity() * 10.0 - ukf->sigma();
	double mag = Eigen::Vector2d::Ones().transpose() * diff * Eigen::Vector2d::Ones();
	this->mag = mag;
	if(mag <= 0.0)
	{
	    ukf.reset();
	}
    }
    
    // estimate structure
    if((inside_estimation_zone && switched_scan_direction) || switched_zone)
    {
	estimated_structure.invalidate();
	cluster_points_debug.clear();
	
	// cluster features
	std::list<base::Vector3d*> feature_list;
	for(std::vector<StructureFeature>::iterator it = features.begin(); it != features.end(); it++)
	{
	    feature_list.push_back(&it->point);
	}
	std::vector< std::set<base::Vector3d*> > clusters;
	if(angular_resolution <= 0.0)
	    sonar_detectors::PointClustering::clusterPointCloud(&feature_list, clusters, 2, dbscan_epsilon);
	else
	{
	    // use angular_resolution as euclidean distance, because for small angles it is almost equal
	    sonar_detectors::PointClustering::clusterPointCloud(&feature_list, clusters, 2, angular_resolution * dbscan_epsilon, false, true);
	}
	
	// for now use biggest cluster
	int best_cluster = -1;
	unsigned best_cluster_size = 0;
	for(unsigned i = 0; i < clusters.size(); i++)
	{
	    if(clusters[i].size() > best_cluster_size)
	    {
		best_cluster_size = clusters[i].size();
		best_cluster = i;
	    }
	}
	if(best_cluster >= 0 && best_cluster_size > min_feature_count)
	{
	    std::vector<base::Vector3d> cluster_points;
	    for(std::set<base::Vector3d*>::iterator it = clusters[best_cluster].begin(); it != clusters[best_cluster].end(); it++)
	    {
		cluster_points.push_back(**it);
	    }
	    cluster_points_debug = cluster_points;
	    
	    // compute center point
	    base::Vector3d center_point = base::Vector3d::Zero();
	    for(std::vector<base::Vector3d>::iterator it = cluster_points.begin(); it != cluster_points.end(); it++)
	    {
		center_point += *it;
	    }
	    center_point /= (double)cluster_points.size();
	    estimated_structure.position = center_point;
	    
	    base::Vector3d auv2cluster = center_point - auvInOdometry.translation();
	    base::Angle angle = base::Angle::fromRad(computeRotation(Eigen::Vector2d(auv2cluster.x(), auv2cluster.y())));
	    Eigen::Quaterniond cluster_rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle.getRad(), Eigen::Vector3d::UnitZ()));
	    estimated_structure.orientation = cluster_rotation;
	    
	    // compute sampled covariance matrix
	    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
	    for(std::vector<base::Vector3d>::iterator it = cluster_points.begin(); it != cluster_points.end(); it++)
	    {
		base::Vector3d diff = *it - center_point;
		cov += diff * diff.transpose();
	    }
	    cov /= (double)cluster_points.size();
	    
	    // compute information based covariance
	    /*
	    if(angular_resolution > 0.0)
	    {
		double structure_opening_angle = atan2(structure_diameter * 0.5, auv2cluster.norm() - (structure_diameter * 0.5)) * 2.0;
		double supposed_measurements = structure_opening_angle / angular_resolution;
		if(supposed_measurements > 0.0)
		{
		    double information = (double)best_cluster_size / supposed_measurements;
		    
		    //std::cerr << "structure_opening_angle: " << structure_opening_angle << " supposed_measurements: " << supposed_measurements << " information : " << information << std::endl;
		    
		    cov = (std::pow(information,2.0) * Eigen::Matrix3d::Identity()).inverse();
		}
	    }
	    */
	    
	    // prediction step
	    if(ukf.get() == NULL)
	    {
		initFilter(center_point, cov);
		candidate_update_count = 0;
	    }
	    else
	    {
		base::Vector2d mu = ukf->mu();
		base::Matrix2d sigma = ukf->sigma();
		
		base::Vector2d diff = center_point.block(0,0,2,1) - mu;
		double mahalanobis_distance = sqrt((diff.transpose() * (sigma + cov.block(0,0,2,2)).inverse() * diff)(0,0));
		
		double diff_norm = diff.norm();
		if((mahalanobis_distance > diff_norm && diff_norm < max_mahalanobis_distance) || 
		    mahalanobis_distance < max_mahalanobis_distance)
		{
		    correctionStep(center_point, cov);
		    candidate_update_count++;
		}
		else
		{
		    initFilter(center_point, cov);
		    candidate_update_count = 0;
		}
	    }
	    
	    // compute structure covariance
	    Eigen::Matrix2d structure_cov = Eigen::Matrix2d::Zero();
	    for(std::vector<base::Vector3d>::iterator it = cluster_points.begin(); it != cluster_points.end(); it++)
	    {
		base::Vector2d diff = it->block(0,0,2,1) - ukf->mu();
		structure_cov += diff * diff.transpose();
	    }
	    structure_cov /= (double)cluster_points.size();
	    estimated_structure.cov_position.setZero();
	    estimated_structure.cov_position.block(0,0,2,2) = structure_cov;
	    
	    // compute cluster cov scaled boundary
	    auv_to_structure_line.clear();
	    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> ev( estimated_structure.cov_position.block(0,0,2,2) );
	    Eigen::Vector2d eigen_values = ev.eigenvalues();
	    Eigen::Vector2d cov_scale = eigen_values.array().sqrt();
	    
	    cluster2cov = auvInOdometry.translation().block(0,0,2,1) - ukf->mu();
	    cluster2cov.normalize();
	    Eigen::Matrix2d scale_matrix = cov_scale.asDiagonal();
	    cluster2cov = scale_matrix * cluster2cov;
	}
	features.clear();
    }
    
    // compute distance to structure
    if(ukf.get() != NULL)
    {
	base::Vector2d auv2cov = (base::Vector2d(ukf->mu().x(), ukf->mu().y()) - auvInOdometry.translation().block(0,0,2,1)) + cluster2cov;
	distance_to_structure = auv2cov.norm();
	
	auv_to_structure_line.push_back(auvInOdometry.translation());
	auv_to_structure_line.push_back(auvInOdometry.translation() + base::Vector3d(auv2cov.x(), auv2cov.y(), 0.0));
    }
}

void StructureEstimation::initFilter(const base::Vector3d& position, const Eigen::Matrix3d &cov)
{
    if(ukf.use_count() > 0)
    {
	ukf.reset();
    }
    
    WStructState state(position.block(0,0,2,1));
    ukf.reset(new UKF(state, cov.block(0,0,2,2)));
}

void StructureEstimation::predictionStep(const double delta)
{
    ukf->predict(boost::bind(processModel<WStructState>, _1), UKF::cov(delta * process_noise_cov));
}

void StructureEstimation::correctionStep(const base::Vector3d& position, const Eigen::Matrix3d &cov)
{
    WStructState new_state(position.block(0,0,2,1));
    
    ukf->update(new_state, boost::bind(measurementModel<WStructState>, _1), 
		boost::bind(ukfom::id<UKF::cov>, cov.block(0,0,2,2)), ukfom::accept_any_mahalanobis_distance<UKF::scalar_type>);
}

base::samples::RigidBodyState StructureEstimation::getStructurePose()
{
    base::samples::RigidBodyState rbs;
    rbs.initUnknown();
    if(ukf.get() != NULL)
    {
	rbs.position.block(0,0,2,1) = ukf->mu();
	rbs.orientation = estimated_structure.orientation;
	rbs.cov_position = estimated_structure.cov_position;
    }
    return rbs;
}

base::samples::RigidBodyState StructureEstimation::getStructureConfidence()
{
    base::samples::RigidBodyState rbs;
    rbs.initUnknown();
    if(ukf.get() != NULL)
    {
	rbs.position.block(0,0,2,1) = ukf->mu();
	rbs.orientation = estimated_structure.orientation;
	rbs.cov_position.setZero();
	rbs.cov_position.block(0,0,2,2) = ukf->sigma();
    }
    return rbs;
}

bool StructureEstimation::isStructureStable()
{
    if(ukf.get() != NULL && candidate_update_count >= min_candidates)
	return true;
    
    return false;
}

}