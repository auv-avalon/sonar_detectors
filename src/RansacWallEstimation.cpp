#include "RansacWallEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include <map>

namespace sonar_detectors
{
    
RansacWallEstimation::RansacWallEstimation()
{
    min_count_pointcloud = 6;
    ransac_threshold = 0.5;
    ransac_fit_rate = 0.7;
    featureList = sonarMap.getFeatureListPtr();
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
        sonarMap.addFeature(featureVector.front(), feature.start_angle, feature.time);

        pointCloud.clear();
        for(std::list< base::Vector3d >::const_iterator l_it = featureList->begin(); l_it != featureList->end(); l_it++)
        {
            // TODO: filter out points which are not between global_start_angle and global_end_angle
            pointCloud.push_back(*l_it);
        }
        
        // check if enough points are available
        if (pointCloud.size() < min_count_pointcloud)
        {
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
            return;
        }
        
        std::vector< std::pair<base::Vector3d, base::Vector3d> > new_walls;
        int iterations = pointCloud.size() * 3;
        if (iterations > 200) iterations = 200;
        double error = sonar_detectors::wallRansac(pointCloud, iterations, ransac_threshold, ransac_fit_rate, new_walls);
        
        if (error < 1.0 && new_walls.size() > 0)
        {
            wall.first = computIntersection(new_walls.front(), base::Vector3d(0,0,0));
            wall.second = new_walls.front().second;
        }
        else
        {
            wall.first = base::Vector3d();
            wall.second = base::Vector3d();
        }
    }
}

void RansacWallEstimation::setRansacParameters(double threshold, double fit_rate)
{
    ransac_fit_rate = fit_rate;
    ransac_threshold = threshold;
}

const std::pair< base::Vector3d, base::Vector3d > RansacWallEstimation::getWall() const
{
    return wall;
}

std::vector<base::Vector3d> RansacWallEstimation::getPointCloud()
{
    return pointCloud;
}
   
}