#include "RansacWallEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include "PointClustering.hpp"

namespace sonar_detectors
{
    
RansacWallEstimation::RansacWallEstimation()
{
    min_count_pointcloud = 6;
    ransac_threshold = 0.5;
    ransac_fit_rate = 0.7;
    dbscan_epsilon = 1.0;
    sonarMap.setFeatureTimeout(8000);
    wall.first = base::Vector3d(0.0,0.0,0.0);
    wall.second = base::Vector3d(0.0,0.0,0.0);
}

RansacWallEstimation::~RansacWallEstimation()
{

}

void RansacWallEstimation::updateFeatureIntern(const base::samples::LaserScan& feature)
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
        sonar_detectors::PointClustering::clusterPointCloud(&pointCloudList, clusters, min_count_pointcloud, 1.0, dbscan_epsilon);
        unsigned cluster_count = 0;
        std::vector< std::set<base::Vector3d*> >::const_iterator best_cluster = clusters.end();
        for(std::vector< std::set<base::Vector3d*> >::const_iterator it = clusters.begin(); it != clusters.end(); it++)
        {
            if(it->size() > cluster_count)
            {
                best_cluster = it;
                cluster_count = it->size();
            }
        }
        pointCloud.clear();
        
        // check if a cluster is available
        if (best_cluster == clusters.end())
        {
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
            return;
        }
        
        for(std::set<base::Vector3d*>::const_iterator it = best_cluster->begin(); it != best_cluster->end(); it++)
        {
            pointCloud.push_back(**it);
        }
        
        // check if enough points are available
        if (pointCloud.size() < min_count_pointcloud)
        {
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
            return;
        }
        
        std::vector< std::pair<base::Vector3d, base::Vector3d> > new_walls;
        unsigned int iterations = pointCloud.size() * 3;
        if (iterations > 200) iterations = 200;
        double error = sonar_detectors::wallRansac(pointCloud, iterations, ransac_threshold, ransac_fit_rate, new_walls);
        
        if (error < 1.0 && new_walls.size() > 0)
        {
            wall.first = computIntersection(new_walls.front(), base::Vector3d(0,0,0));
            wall.second = new_walls.front().second;
        }
        else
        {
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
        }
    }
}

void RansacWallEstimation::setRansacParameters(double ransac_threshold, double ransac_fit_rate, double dbscan_epsilon)
{
    this->ransac_fit_rate = ransac_fit_rate;
    this->ransac_threshold = ransac_threshold;
    this->dbscan_epsilon = dbscan_epsilon;
}

const std::pair< base::Vector3d, base::Vector3d >& RansacWallEstimation::getWall() const
{
    return wall;
}

const std::vector<base::Vector3d>& RansacWallEstimation::getPointCloud()
{
    return pointCloud;
}
   
}