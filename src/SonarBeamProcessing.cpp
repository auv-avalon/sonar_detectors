#include "SonarBeamProcessing.hpp"

namespace avalon
{
SonarBeamProcessing::SonarBeamProcessing(BeamMode beamMode, PersistMode displayMode)
{
    this->beamMode = beamMode;
    this->persistMode = displayMode;
    minThreshold = 0;
    maxThreshold = 100;
    enableThreshold = false;
    estimateWalls = false;
    newOrientationOrPosition = false;
    minResponseValue = 0;
    position.setZero();
    orientation = Eigen::Quaterniond::Identity();
    
    // create initial segment
    scanSegment segment;
    segments.push_back(segment);
    
    inputCount = 0;
}

SonarBeamProcessing::~SonarBeamProcessing()
{
    
}

void SonarBeamProcessing::selectBeamMode(BeamMode mode)
{
    beamMode = mode;
}

void SonarBeamProcessing::selectPersistMode(PersistMode mode)
{
    persistMode = mode;
}

const std::vector<scanSegment>* SonarBeamProcessing::getPoints() const
{
    return &segments;
}

void SonarBeamProcessing::updateOrientation(base::Orientation& orientation)
{
    this->orientation = orientation;
    newOrientationOrPosition = true;
}

void SonarBeamProcessing::updatePosition(base::Position& position)
{
    this->position = position;
    newOrientationOrPosition = true;
}

void SonarBeamProcessing::enableBeamThreshold(bool b)
{
    this->enableThreshold = b;
}

void SonarBeamProcessing::setBeamThreshold(double minThreshold, double maxThreshold)
{
    if(minThreshold >= 0 && maxThreshold >= 0)
    {
        this->minThreshold = minThreshold;
        this->maxThreshold = maxThreshold;
    }
    else
    {
        std::cerr << "The threshold values must be positiv defined!" << std::endl;
    }
}

void SonarBeamProcessing::enableWallEstimation(bool b)
{
    this->estimateWalls = b;
}

void SonarBeamProcessing::setMinResponseValue(unsigned int minValue)
{
    this->minResponseValue = minValue;
}

void SonarBeamProcessing::persistPoints(const std::vector<obstaclePoint>& obstaclePoints, double& angle, scanSegment& segment)
{
    std::list<obstaclePoint>& sortedPoints = segment.pointCloud;
    std::list<obstaclePoint>::iterator& listIterator = segment.latestBeam;
    switch (persistMode)
    {
        case persistNewScans:
        {
            std::vector<obstaclePoint>::const_iterator beginIt = obstaclePoints.begin();
            std::vector<obstaclePoint>::const_iterator endIt = obstaclePoints.end();
            // init empty list
            if (sortedPoints.empty())
            {
                listIterator = sortedPoints.end();
                sortedPoints.insert(listIterator, beginIt, endIt);
                listIterator--;
                return;
            }
            // handle switch between 0 and 2 PI
            if (std::abs(segment.latestBeam->angle - angle) > M_PI)
            {
                if (angle > M_PI) 
                {
                    segment.latestBeam = segment.pointCloud.end();
                    segment.latestBeam--;
                }
                else 
                    segment.latestBeam = segment.pointCloud.begin();
            }
            // insert new points
            std::list<obstaclePoint>::iterator shiftedListIterator(listIterator);
            // points with equal angle
            if (angle == listIterator->angle)
            {
                // delete equal points
                shiftedListIterator--;
                while(listIterator != sortedPoints.begin() && angle == shiftedListIterator->angle)
                {
                    listIterator--;
                    shiftedListIterator--;
                }
                while(listIterator != sortedPoints.end() && angle == listIterator->angle)
                {
                    listIterator = sortedPoints.erase(listIterator);
                }
                // add new points
                sortedPoints.insert(listIterator, beginIt, endIt);
                listIterator--;
            }
            else if (angle > listIterator->angle)
            {
                shiftedListIterator++;
                while(shiftedListIterator != sortedPoints.end() && listIterator->angle == shiftedListIterator->angle)
                {
                    listIterator++;
                    shiftedListIterator++;
                }
                listIterator++;
                while(listIterator != sortedPoints.end() && angle >= listIterator->angle)
                {
                    listIterator = sortedPoints.erase(listIterator);
                }
                sortedPoints.insert(listIterator, beginIt, endIt);
                listIterator--;
            }
            else if (angle < listIterator->angle)
            {
                shiftedListIterator--;
                while(listIterator != sortedPoints.begin() && listIterator->angle == shiftedListIterator->angle)
                {
                    listIterator--;
                    shiftedListIterator--;
                }
                while(listIterator != sortedPoints.begin() && angle <= shiftedListIterator->angle)
                {
                    shiftedListIterator = sortedPoints.erase(shiftedListIterator);
                    shiftedListIterator--;
                }
                sortedPoints.insert(listIterator, beginIt, endIt);
                listIterator--;
            }
        }
        break;
        case persistAll:
            for(std::vector<obstaclePoint>::const_iterator it = obstaclePoints.begin(); it != obstaclePoints.end(); it++)
            {
                sortedPoints.push_back(*it);
            }
            break;
    }
}

std::vector<int> SonarBeamProcessing::computeSonarScanIndex(const std::vector<base::samples::SonarScan::uint8_t>& scan, const unsigned& minIndex, const unsigned& maxIndex, const unsigned& minValue)
{
    std::vector<int> indexList;
    base::samples::SonarScan::uint8_t data = minValue;
    int index = minIndex - 1;
    int nextIndex = 0;
    switch (beamMode)
    {
        case globalMaximum:
            for(unsigned i = minIndex; i < maxIndex && i < scan.size(); i++)
            {
                if(scan[i] > data)
                {
                    data = scan[i];
                    index = i;
                }
            }
            indexList.push_back(index);
            break;
        case firstLokalMaximum:
            index = getNextMaximum(index + 1, maxIndex, minValue, scan);
            indexList.push_back(index);
            break;
        case allLokalMaxima:
            do 
            {
                nextIndex = index + 1;
                index = getNextMaximum(nextIndex, maxIndex, minValue, scan);
                if (nextIndex != index) indexList.push_back(index);
            } 
            while(nextIndex != index);
            break;
        case allEntries:
            for(unsigned i = minIndex; i < maxIndex && i < scan.size(); i++)
            {
                if(scan[i] > 0)
                {
                    indexList.push_back(i);
                }
            }
            break;
    }
    return indexList;
}

//NOTE: maximas actually have to be greater in each following step
int SonarBeamProcessing::getNextMaximum(int startIndex, int endIndex, unsigned minValue, const std::vector<base::samples::SonarScan::uint8_t>& scan) 
{
    base::samples::SonarScan::uint8_t data = scan[startIndex] < minValue ? minValue : scan[startIndex];
    int index = startIndex;
    for(unsigned i = startIndex + 1; i < endIndex && i < scan.size(); i++)
    {
        if(scan[i] > data)
        {
            data = scan[i];
            index = i;
        }
        else if (index != startIndex && scan[i] < data) break;
    }
    return index;
}

obstaclePoint SonarBeamProcessing::computeObstaclePoint(const int& index, const base::samples::SonarScan& sonarScan)
{
    base::Time scanTime = sonarScan.time;
    double scanAngle = sonarScan.angle;
    double time_beetween_bins = sonarScan.time_beetween_bins;
    std::vector<base::samples::SonarScan::uint8_t> scan = sonarScan.scanData;
    
    double distance = (index * time_beetween_bins * sonicVelocityInWater) / 2;
    std::cout << "Time: " << scanTime << ", Angle: " << scanAngle << ", Distance: " << distance << ", Value: " << (uint)scan[index] << std::endl;
        
    Eigen::Vector3d wallPoint(-distance,0,0);
    Eigen::Vector3d topPoint(0,0,1);
    
    wallPoint = orientation * wallPoint;
    topPoint = orientation * topPoint;
    
    Eigen::AngleAxisd rotate(-scanAngle,topPoint);
    
    Eigen::Translation3d translate(position);
    wallPoint = translate * (rotate * wallPoint);
        
    avalon::obstaclePoint obstaclePoint;
    obstaclePoint.position = wallPoint;
    obstaclePoint.time = scanTime;
    obstaclePoint.value = scan[index];
    obstaclePoint.angle = scanAngle;
    
    return obstaclePoint;
}

void SonarBeamProcessing::computeWall(avalon::scanSegment& segment)
{
    if (segment.pointCloud.size() > 2) 
    {
        segment.walls.clear();
        if(segment.latestBeam != segment.pointCloud.end())
        {
            std::vector<cv::Point3f> cvPoints1;
            for(std::list<avalon::obstaclePoint>::iterator it = segment.pointCloud.begin(); it != segment.latestBeam; it++)
            {
                base::Position& pos = it->position;
                cvPoints1.push_back(cv::Point3f(pos.x(), pos.y(), pos.z()));
            }
            
            if(cvPoints1.size() >= 10) 
            {
                cv::Vec6f line1;
                cv::Mat mat1 = cv::Mat(cvPoints1);
                cv::fitLine(mat1, line1, CV_DIST_L2, 0, 0.01, 0.01);
                
                std::pair<base::Position, base::Position> wall1(base::Position(line1[3], line1[4], line1[5]), base::Position(line1[0], line1[1], line1[2]));
                segment.walls.push_back(wall1);
            }
            
            cv::vector<cv::Point3f> cvPoints2;
            for(std::list<avalon::obstaclePoint>::iterator it = segment.latestBeam; it != segment.pointCloud.end(); it++)
            {
                base::Position& pos = it->position;
                cvPoints2.push_back(cv::Point3d(pos.x(), pos.y(), pos.z()));
            }
            
            if(cvPoints2.size() >= 10)
            {
                cv::Vec6f line2;
                cv::Mat mat2(cvPoints2);
                cv::fitLine(mat2, line2, CV_DIST_L2, 0, 0.01, 0.01);
                
                std::pair<base::Position, base::Position> wall2(base::Position(line2[3], line2[4], line2[5]), base::Position(line2[0], line2[1], line2[2]));
                segment.walls.push_back(wall2);
            }
        } 
        else
        {
            std::vector<cv::Point3f> cvPoints;
            for(std::list<avalon::obstaclePoint>::iterator it = segment.pointCloud.begin(); it != segment.pointCloud.end(); it++)
            {
                base::Position& pos = it->position;
                cvPoints.push_back(cv::Point3f(pos.x(), pos.y(), pos.z()));
            }
            
            if(cvPoints.size() >= 2) 
            {
                cv::Vec6f line;
                cv::Mat mat = cv::Mat(cvPoints);
                std::cout << "cv::Mat: " << mat.elemSize() << " " << mat.isContinuous() << std::endl;
                cv::fitLine(mat, line, CV_DIST_L2, 0, 0.01, 0.01);
                
                std::pair<base::Position, base::Position> wall(base::Position(line[3], line[4], line[5]), base::Position(line[0], line[1], line[2]));
                segment.walls.push_back(wall);
            }
        }
    }
}

void SonarBeamProcessing::computeVirtualPoint(scanSegment& segment)
{
    if (segment.walls.size() == 1)
    {
        std::pair<base::Vector3d, base::Vector3d> wall = segment.walls[0];
        double lambda = position.x() * wall.second.x() + position.y() * wall.second.y() + position.z() * wall.second.z();
        double x = wall.second.x() * wall.first.x() + wall.second.y() * wall.first.y() + wall.second.z() * wall.first.z();
        double y = wall.second.x() * wall.second.x() + wall.second.y() * wall.second.y() + wall.second.z() * wall.second.z();
        lambda -= x;
        lambda /= y;
        segment.virtualpoint = wall.first + (lambda * wall.second);
    }
    else
    {
        segment.virtualpoint = base::Vector3d(0,0,0);
    }
}

base::Vector3d SonarBeamProcessing::getVirtualPoint() const
{
    //TODO: make this better
    if (segments.size() > 0 && estimateWalls)
    {
        base::Vector3d vpos(segments.back().virtualpoint);
        vpos -= position;
        vpos = orientation.conjugate() * vpos;
        return vpos;
    }
    else 
        return base::Vector3d(0,0,0);
}

void SonarBeamProcessing::createNewSegment()
{
    scanSegment segment;
    computeWall(segments.back());
    segments.push_back(segment);
    //if(maximumSegmentCount < segments.size()) {
    //    segments.erase(segments.begin());
    //}
}

void SonarBeamProcessing::updateSonarData(const base::samples::SonarScan& sonarScan)
{
    std::vector<base::samples::SonarScan::uint8_t> scan = sonarScan.scanData;
    double scanAngle = sonarScan.angle;
    double time_beetween_bins = sonarScan.time_beetween_bins;
    
    std::vector<int> indexList;
    // analyze index
    if (enableThreshold && (time_beetween_bins * sonicVelocityInWater > 0))
    {
        //cut scan index
        int minIndex = (minThreshold * 2) / (time_beetween_bins * sonicVelocityInWater);
        int maxIndex = (maxThreshold * 2) / (time_beetween_bins * sonicVelocityInWater);
        indexList = computeSonarScanIndex(scan, minIndex, maxIndex, minResponseValue);
    }
    else
    {
        indexList = computeSonarScanIndex(scan, 0, scan.size(), minResponseValue); 
    }
    
    std::vector<obstaclePoint> obstaclePoints;
    if (!indexList.empty())
    {
        // calculate points
        while (indexList.size() > 0)
        {
            avalon::obstaclePoint obstaclePoint = computeObstaclePoint(indexList.back(), sonarScan);
            indexList.pop_back();
            obstaclePoints.push_back(obstaclePoint);
        }
    }
    persistPoints(obstaclePoints, scanAngle, segments.back());
    
    // estimate walls
    scanSegment& activSegment = segments.back();
    if (estimateWalls)
    {
        std::list<obstaclePoint>::iterator endIt = activSegment.pointCloud.end();
        endIt--;
        
        if (activSegment.pointCloud.begin() == activSegment.latestBeam || 
            endIt == activSegment.latestBeam)// || activSegment.latestBeam->angle < 3.14 && activSegment.latestBeam->angle >= 3.1)
        {
            computeWall(activSegment);
            computeVirtualPoint(activSegment);
        }
        else if (newOrientationOrPosition && activSegment.walls.size())
        {
            computeVirtualPoint(activSegment);
        }
    }
    else
    {
        if (activSegment.walls.size() > 0)
        {
            activSegment.walls.clear();
            activSegment.virtualpoint = base::Vector3d(0,0,0);
        }
    }
}

}
