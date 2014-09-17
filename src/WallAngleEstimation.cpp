#include "WallAngleEstimation.hpp"
#include <sonar_detectors/SonarDetectorMath.hpp>

namespace sonar_detectors
{
    
WallAngleEstimation::WallAngleEstimation() : min_feature_count(10), ransac_threshold(0.4), ransac_fit_rate(0.4), wall_candidate_count(3), max_angle_sigma(0.1), max_distance_sigma(0.2)
{
    angle_to_wall.rad = base::NaN<double>();
    wall_angle.rad = base::NaN<double>();
    distance_to_wall = base::NaN<double>();
}

void WallAngleEstimation::setRansacParameters(double ransac_threshold, double ransac_fit_rate)
{
    this->ransac_threshold = ransac_threshold;
    this->ransac_fit_rate = ransac_fit_rate;
}

void WallAngleEstimation::setStabilityParameters(unsigned wall_candidate_count, double max_angle_sigma, double max_distance_sigma)
{
    this->wall_candidate_count = wall_candidate_count;
    this->max_angle_sigma = max_angle_sigma;
    this->max_distance_sigma = max_distance_sigma;
}

bool WallAngleEstimation::getWallAngle(base::Angle &angle)
{
    if(base::isNaN<double>(wall_angle.rad))
	return false;
    
    angle = wall_angle;
    return true;
}

bool WallAngleEstimation::getAngleToWall(base::Angle& angle)
{
    if(base::isNaN<double>(angle_to_wall.rad))
	return false;
    
    angle = angle_to_wall;
    return true;
}

bool WallAngleEstimation::getDistanceToWall(double& distance)
{
    if(base::isNaN<double>(distance_to_wall))
	return false;
    
    distance = distance_to_wall;
    return true;
}

void WallAngleEstimation::updateFeatureIntern(const base::samples::LaserScan& feature, const Eigen::Affine3d &featureInOdometry)
{
    std::vector<Eigen::Vector3d> featureVector;
    feature.convertScanToPointCloud(featureVector, featureInOdometry);
    
    if(featureVector.size() > 0)
    {
        // add new feature
        sonarMap.addFeature(featureVector.front(), feature.start_angle + global_heading.getRad(), feature.time);
	
	if(switched_scan_direction || switched_zone)
	{
	    // get sub features in estimation zone
	    std::vector<base::Vector3d> featureCloud;
	    sonarMap.getSubFeatureVector(featureCloud, global_start_angle.rad, global_end_angle.rad);
	    debug_feature_cloud = featureCloud;
	    
	    std::pair<base::Vector3d, base::Vector3d> line;
	    if(estimateLine(featureCloud, line))
	    {
		WallCandidate candidate;
		candidate.angle_to_wall = base::Angle::fromRad(computeAngle(base::Vector3d(1.0,0,0), line.first));
		base::Angle local_wall_angle = base::Angle::fromRad(computeAngle(base::Vector3d(1.0,0,0), line.second));
		if(abs(local_wall_angle.rad) > M_PI_2)
		    local_wall_angle += base::Angle::fromRad(M_PI);
		candidate.wall_angle = local_wall_angle;
		candidate.wall_distance = length(line.first);
		
		wall_candidates.push_back(candidate);
		if(wall_candidates.size() > wall_candidate_count)
		    wall_candidates.erase(wall_candidates.begin());
		
		WallCandidate mean_candidate;
		if(computeMean(mean_candidate))
		{
		    angle_to_wall = mean_candidate.angle_to_wall;
		    wall_angle = mean_candidate.wall_angle;
		    return;
		}
		else
		{
		    // reset angles
		    angle_to_wall.rad = base::NaN<double>();
		    wall_angle.rad = base::NaN<double>();
		    distance_to_wall = base::NaN<double>();
		    return;
		}
	    }
	    
	    // reset angles and candidates
	    wall_candidates.clear();
	    angle_to_wall.rad = base::NaN<double>();
	    wall_angle.rad = base::NaN<double>();
	    distance_to_wall = base::NaN<double>();
	}
    }
    	// TODO: add time delta to sonar map if we don't get new features
}

bool WallAngleEstimation::estimateLine(const std::vector< base::Vector3d >& point_cloud, std::pair<base::Vector3d, base::Vector3d>& line)
{
    if(point_cloud.size() >= min_feature_count)
    {
	std::vector< std::pair<base::Vector3d, base::Vector3d> > best_lines;
	// the amount of needed iterations depends only on the relative amount of outliers. 50 should be already enough for 70% outliers.
	double iterations = 100;
	double error = sonar_detectors::wallRansac(point_cloud, iterations, ransac_threshold, ransac_fit_rate, best_lines);
	
	if (error < 1.0 && best_lines.size() > 0)
	{
	    // TODO maybe do a linear line optimization on the inlier set
	    
	    line.first = computIntersection(best_lines.front(), base::Vector3d(0,0,0));
	    line.second = best_lines.front().second;
	    
	}
	
	return true;
    }
    
    return false;
}

bool WallAngleEstimation::computeMean(WallCandidate& mean)
{
    mean.angle_to_wall.rad = 0.0;
    mean.wall_angle.rad = 0.0;
    mean.wall_distance = 0;
    if(wall_candidates.size() == wall_candidate_count)
    {
	// compute means
	for(unsigned i = 0; i < wall_candidates.size(); i++)
	{
	    mean.angle_to_wall.rad += wall_candidates[i].angle_to_wall.rad;
	    mean.wall_angle.rad += wall_candidates[i].wall_angle.rad;
	    mean.wall_distance += wall_candidates[i].wall_distance;
	}
	mean.angle_to_wall = base::Angle::fromRad(mean.angle_to_wall.rad / (double)wall_candidates.size());
	mean.wall_angle = base::Angle::fromRad(mean.wall_angle.rad / (double)wall_candidates.size());
	mean.wall_distance = mean.wall_distance / (double)wall_candidates.size();
	
	// compute sigmas
	double angle_sigma = 0.0;
	double distance_sigma = 0.0;
	for(unsigned i = 0; i < wall_candidates.size(); i++)
	{
	    angle_sigma += pow((wall_candidates[i].wall_angle - mean.wall_angle).getRad(), 2.0);
	    distance_sigma += pow(wall_candidates[i].wall_distance - mean.wall_distance , 2.0);
	}
	angle_sigma /= (double)wall_candidates.size();
	distance_sigma /= (double)wall_candidates.size();
	angle_sigma = sqrt(angle_sigma);
	distance_sigma = sqrt(distance_sigma);
	
	if(angle_sigma < max_angle_sigma && distance_sigma < max_distance_sigma)
	    return true;
    }
    
    return false;
}
    
}