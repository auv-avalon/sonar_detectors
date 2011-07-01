#include "WallEstimation.hpp"
#include "SonarDetectorMath.hpp"

namespace avalon
{
    
WallEstimation::WallEstimation()
{
    min_count_pointcloud = 6;
    ransac_threshold = 0.5;
    ransac_fit_rate = 0.7;
}

WallEstimation::~WallEstimation()
{

}

void WallEstimation::updateSegment(const avalon::scanSegment& segment)
{
    // check if enough points available
    if (segment.pointCloud.size() < min_count_pointcloud)
    {
        walls.clear();
        return;
    }
    
    std::vector<base::Vector3d> pointCloud;
    for(std::list<obstaclePoint>::const_iterator it = segment.pointCloud.begin(); it != segment.pointCloud.end(); it++)
    {
        pointCloud.push_back(it->position);
    }
    
    std::vector< std::pair<base::Vector3d, base::Vector3d> > new_walls;
    int iterations = pointCloud.size() * 3;
    if (iterations > 200) iterations = 200;
    double error = avalon::wallRansac(pointCloud, iterations, ransac_threshold, ransac_fit_rate, new_walls);
    
    if (error < 1.0)
    {
        walls = new_walls;
    }
    else
    {
        walls.clear();
    }
}

void WallEstimation::computeVirtualPoint()
{
    if (walls.size() == 1)
    {
        virtualpoint = computIntersection(walls[0], *position);
    }
    else if (walls.size() == 2)
    {
        base::Vector3d vp1 = computIntersection(walls[0], *position);
        base::Vector3d vp2 = computIntersection(walls[1], *position);
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

const base::Vector3d WallEstimation::getRelativeVirtualPoint()
{
    computeVirtualPoint();
    if (virtualpoint.x() == 0 && virtualpoint.y() == 0 && virtualpoint.z() == 0)
    {
        return virtualpoint;
    }
    else
    {
        base::Vector3d vpos(virtualpoint);
        vpos -= *position;
        vpos = orientation->conjugate() * vpos;
        return vpos;
    }
}   
   
}