#include "WallEstimation.hpp"
#include <opencv/cv.h>

namespace avalon
{
    
WallEstimation::WallEstimation()
{

}

WallEstimation::~WallEstimation()
{

}

void WallEstimation::updateSegment(const avalon::scanSegment& segment)
{
    computeWall(segment.pointCloud, segment.latestBeam);
}

void WallEstimation::computeWall(const std::list<obstaclePoint>& pointCloud, std::list<obstaclePoint>::const_iterator latestBeam)
{
    if (pointCloud.size() > 2) 
    {
        walls.clear();
        if(latestBeam != pointCloud.end())
        {
            std::vector<cv::Point3f> cvPoints1;
            for(std::list<avalon::obstaclePoint>::const_iterator it = pointCloud.begin(); it != latestBeam; it++)
            {
                const base::Position& pos = it->position;
                cvPoints1.push_back(cv::Point3f(pos.x(), pos.y(), pos.z()));
            }
            
            if(cvPoints1.size() >= 10) 
            {
                cv::Vec6f line1;
                cv::Mat mat1 = cv::Mat(cvPoints1);
                cv::fitLine(mat1, line1, CV_DIST_L2, 0, 0.01, 0.01);
                
                std::pair<base::Position, base::Position> wall1(base::Position(line1[3], line1[4], line1[5]), base::Position(line1[0], line1[1], line1[2]));
                walls.push_back(wall1);
            }
            
            cv::vector<cv::Point3f> cvPoints2;
            for(std::list<avalon::obstaclePoint>::const_iterator it = latestBeam; it != pointCloud.end(); it++)
            {
                const base::Position& pos = it->position;
                cvPoints2.push_back(cv::Point3d(pos.x(), pos.y(), pos.z()));
            }
            
            if(cvPoints2.size() >= 10)
            {
                cv::Vec6f line2;
                cv::Mat mat2(cvPoints2);
                cv::fitLine(mat2, line2, CV_DIST_L2, 0, 0.01, 0.01);
                
                std::pair<base::Position, base::Position> wall2(base::Position(line2[3], line2[4], line2[5]), base::Position(line2[0], line2[1], line2[2]));
                walls.push_back(wall2);
            }
        } 
        else
        {
            std::vector<cv::Point3f> cvPoints;
            for(std::list<avalon::obstaclePoint>::const_iterator it = pointCloud.begin(); it != pointCloud.end(); it++)
            {
                const base::Position& pos = it->position;
                cvPoints.push_back(cv::Point3f(pos.x(), pos.y(), pos.z()));
            }
            
            if(cvPoints.size() >= 2) 
            {
                cv::Vec6f line;
                cv::Mat mat = cv::Mat(cvPoints);
                std::cout << "cv::Mat: " << mat.elemSize() << " " << mat.isContinuous() << std::endl;
                cv::fitLine(mat, line, CV_DIST_L2, 0, 0.01, 0.01);
                
                std::pair<base::Position, base::Position> wall(base::Position(line[3], line[4], line[5]), base::Position(line[0], line[1], line[2]));
                walls.push_back(wall);
            }
        }
    }
}

void WallEstimation::computeVirtualPoint()
{
    if (walls.size() == 1)
    {
        std::pair<base::Vector3d, base::Vector3d> wall = walls[0];
        double lambda = position->x() * wall.second.x() + position->y() * wall.second.y() + position->z() * wall.second.z();
        double x = wall.second.x() * wall.first.x() + wall.second.y() * wall.first.y() + wall.second.z() * wall.first.z();
        double y = wall.second.x() * wall.second.x() + wall.second.y() * wall.second.y() + wall.second.z() * wall.second.z();
        lambda -= x;
        lambda /= y;
        virtualpoint = wall.first + (lambda * wall.second);
    }
    else
    {
        virtualpoint = base::Vector3d(0,0,0);
    }
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