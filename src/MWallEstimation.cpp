#include "MWallEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include "PointClustering.hpp"

namespace sonar_detectors
{
    
MWallEstimation::MWallEstimation()
{
    min_count_pointcloud = 6;
    min_wall_distance = 1.0;
    line_fit_threshold = 1.0;
    min_fit_rate = 0.7;
    dbscan_epsilon = 1.0;
    sonarMap.setFeatureTimeout(8000);
    wall.first = base::Vector3d(0.0,0.0,0.0);
    wall.second = base::Vector3d(0.0,0.0,0.0);
    angular_resolution = 0.0;
}

MWallEstimation::~MWallEstimation()
{

}

void MWallEstimation::updateFeatureIntern(const base::samples::LaserScan& feature)
{
    std::vector<Eigen::Vector3d> featureVector;
    feature.convertScanToPointCloud(featureVector);
    
    if(featureVector.size() > 0)
    {
        // add new feature
        sonarMap.addFeature(featureVector.front(), feature.start_angle, feature.time);

        // get sub features in estimation zone
        std::vector<base::Vector3d> featureCloud;
        sonarMap.getSubFeatureVector(featureCloud, global_start_angle.rad, global_end_angle.rad);
        
        // cluster features
        std::list<base::Vector3d*> pointCloudList;
        for(std::vector<base::Vector3d>::iterator it = featureCloud.begin(); it != featureCloud.end(); it++)
        {
            pointCloudList.push_back(&*it);
        }
        std::vector< std::set<base::Vector3d*> > clusters;
        if(angular_resolution <= 0.0)
            sonar_detectors::PointClustering::clusterPointCloud(&pointCloudList, clusters, 2, dbscan_epsilon);
        else    
        {
            // use angular_resolution as euclidean distance, because for small angles it is almost equal
            sonar_detectors::PointClustering::clusterPointCloud(&pointCloudList, clusters, 2, dbscan_epsilon / angular_resolution, false, true, angular_resolution);
        }
        std::vector< wallCandidate > wall_candidates;
        for(std::vector< std::set<base::Vector3d*> >::const_iterator it = clusters.begin(); it != clusters.end(); it++)
        {
            // check if the cluster has enough points
            if(it->size() >= min_count_pointcloud)
            {
                wallCandidate candidate;
                for(std::set<base::Vector3d*>::const_iterator it2 = it->begin(); it2 != it->end(); it2++)
                {
                    candidate.point_cloud.push_back(**it2);
                }
                wall_candidates.push_back(candidate);
            }
        }
        
        // check if a cluster is available
        if (wall_candidates.size() == 0)
        {
            pointCloud.clear();
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
            return;
        }
        
        // compute candidates line, line distance and fit rate
        std::vector<base::Vector3d> inlier, outlier;
        for(std::vector< wallCandidate >::iterator it = wall_candidates.begin(); it != wall_candidates.end(); it++)
        {
            inlier.clear();
            outlier.clear();
            sonar_detectors::computeLine(it->point_cloud, it->line);
            it->line_distance = sonar_detectors::length(it->line.first);
            it->fit_rate = sonar_detectors::lineFitEvaluation(it->point_cloud, it->line, line_fit_threshold, inlier, outlier);
        }
        
        // find best match
        double wall_distance = min_wall_distance;
        int best_index = -1;
        for(unsigned int i = 0; i < wall_candidates.size(); i++)
        {
            if(wall_candidates[i].line_distance > wall_distance && wall_candidates[i].fit_rate > min_fit_rate)
            {
                wall_distance = wall_candidates[i].line_distance;
                best_index = (int)i;
            }
        }

        pointCloud.clear();
        if(best_index >= 0)
        {
            wall = wall_candidates[best_index].line;
            pointCloud = wall_candidates[best_index].point_cloud;
        }
        else
        {
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
        }
    }
}

void MWallEstimation::setParameters(double line_fit_threshold, double min_fit_rate, double dbscan_epsilon, double angular_resolution)
{
    this->min_fit_rate = min_fit_rate;
    this->line_fit_threshold = line_fit_threshold;
    this->dbscan_epsilon = dbscan_epsilon;
    this->angular_resolution = angular_resolution;
}

const std::pair< base::Vector3d, base::Vector3d >& MWallEstimation::getWall() const
{
    return wall;
}

const std::vector<base::Vector3d>& MWallEstimation::getPointCloud()
{
    return pointCloud;
}

std::vector< base::Vector3d > MWallEstimation::getCompletePointCloud()
{
    std::vector<base::Vector3d> featureCloud;
    sonarMap.getSubFeatureVector(featureCloud, global_start_angle.rad, global_end_angle.rad);
    return featureCloud;
}
   
}