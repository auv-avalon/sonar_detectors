#include "WallEstimation.hpp"
#include "SonarDetectorMath.hpp"

namespace sonar_detectors
{
    
WallEstimation::WallEstimation()
{
    min_count_pointcloud = 6;
    ransac_threshold = 0.5;
    ransac_fit_rate = 0.7;
    featureList = sonarMap.getFeatureListPtr();
}

WallEstimation::~WallEstimation()
{

}

void WallEstimation::updateFeatureIntern(const base::samples::LaserScan& feature)
{
    std::vector<Eigen::Vector3d> featureVector;
    feature.convertScanToPointCloud(featureVector);
    
    if(featureVector.size() > 0)
    {
        sonarMap.addFeature(featureVector.front(), feature.start_angle, feature.time);
        
        // check if enough points available
        if (featureList->size() < min_count_pointcloud)
        {
            walls.clear();
            return;
        }

        std::vector<base::Vector3d> pointCloud;
        for(std::list< base::Vector3d >::const_iterator l_it = featureList->begin(); l_it != featureList->end(); l_it++)
        {
            pointCloud.push_back(*l_it);
        }
        
        std::vector< std::pair<base::Vector3d, base::Vector3d> > new_walls;
        int iterations = pointCloud.size() * 3;
        if (iterations > 200) iterations = 200;
        double error = sonar_detectors::wallRansac(pointCloud, iterations, ransac_threshold, ransac_fit_rate, new_walls);
        
        if (error < 1.0)
        {
            walls = new_walls;
        }
        else
        {
            walls.clear();
        }
    }
}

void WallEstimation::computeVirtualPoint()
{
    if (walls.size() == 1)
    {
        virtualpoint = computIntersection(walls[0], base::Vector3d(0,0,0));
    }
    else if (walls.size() == 2)
    {
        base::Vector3d vp1 = computIntersection(walls[0], base::Vector3d(0,0,0));
        base::Vector3d vp2 = computIntersection(walls[1], base::Vector3d(0,0,0));
        virtualpoint = vp1 + 0.5 * (vp2 - vp1);
    }
    else
    {
        virtualpoint = base::Vector3d(0,0,0);
    }
}

void WallEstimation::setRansacParameters(double threshold, double fit_rate)
{
    ransac_fit_rate = fit_rate;
    ransac_threshold = threshold;
}

const std::vector< std::pair< base::Vector3d, base::Vector3d > > WallEstimation::getWalls() const
{
    return walls;
}

const base::Vector3d WallEstimation::getVirtualPoint()
{
    computeVirtualPoint();
    return virtualpoint;
}

std::list<base::Vector3d> WallEstimation::getPointCloud()
{
    return sonarMap.getFeatureList();
}
   
}