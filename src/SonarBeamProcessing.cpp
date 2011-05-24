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
    minResponseValue = 0;
    position.setZero();
    orientation = Eigen::Quaterniond::Identity();
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

void SonarBeamProcessing::updateOrientation(base::Orientation& orientation)
{
    this->orientation = orientation;
}

void SonarBeamProcessing::updatePosition(base::Position& position)
{
    this->position = position;
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

void SonarBeamProcessing::setMinResponseValue(int minValue)
{
    if (minValue < 0)
        this->minResponseValue = 0;
    else if (minValue > 255)
        this->minResponseValue = 255;
    else
        this->minResponseValue = minValue;
}

void SonarBeamProcessing::addSonarEstimation(SonarEstimation* estimation)
{
    estimation->setPose(&orientation, &position);
    estimator est;
    est.estimation = estimation;
    est.settings = estimation->getSettings();
    estimators.push_back(est);
}

void SonarBeamProcessing::removeSonarEstimation(SonarEstimation* estimation)
{
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if (it->estimation == estimation)
        {
            estimators.erase(it);
        }
    }
}

void SonarBeamProcessing::persistPoints(const std::vector<obstaclePoint>& obstaclePoints, const double& angle, scanSegment& segment)
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

std::vector<int> SonarBeamProcessing::computeSonarScanIndex(const std::vector<base::samples::SonarScan::uint8_t>& scan, const int& minIndex, const int& maxIndex, const int& minValue)
{
    std::vector<int> indexList;
    base::samples::SonarScan::uint8_t data = (uint8_t)minValue;
    int index = -1;
    int nextIndex = 0;
    switch (beamMode)
    {
        case globalMaximum:
            for(int i = minIndex; i < maxIndex && i < scan.size(); i++)
            {
                if(scan[i] > data)
                {
                    data = scan[i];
                    index = i;
                }
            }
            if (index >= 0)
                indexList.push_back(index);
            break;
        case firstLokalMaximum:
            index = getNextMaximum(minIndex, maxIndex, minValue, scan);
            indexList.push_back(index);
            break;
        case allLokalMaxima:
            index = minIndex - 1;
            do 
            {
                nextIndex = index + 1;
                index = getNextMaximum(nextIndex, maxIndex, minValue, scan);
                if (nextIndex != index) indexList.push_back(index);
            } 
            while(nextIndex != index);
            break;
        case allEntries:
            for(int i = minIndex; i < maxIndex && i < scan.size(); i++)
            {
                if(scan[i] > minValue)
                {
                    indexList.push_back(i);
                }
            }
            break;
    }
    return indexList;
}

//NOTE: maximas actually have to be greater in each following step
int SonarBeamProcessing::getNextMaximum(const int& startIndex, const int& endIndex, const int& minValue, const std::vector<base::samples::SonarScan::uint8_t>& scan) 
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
    
    double distance = (index * time_beetween_bins * sonicVelocityInWater) / 2 / 10;
    //std::cout << "Time: " << scanTime << ", Angle: " << scanAngle << ", Distance: " << distance << ", Value: " << (uint)scan[index] << std::endl;
        
    Eigen::Vector3d wallPoint(-distance,0,0);
    Eigen::Vector3d topPoint(0,0,1);
    
    wallPoint = orientation * wallPoint;
    topPoint = orientation * topPoint;
    
    Eigen::AngleAxisd rotate(scanAngle,topPoint);
    
    Eigen::Translation3d translate(position);
    wallPoint = translate * (rotate * wallPoint);
        
    avalon::obstaclePoint obstaclePoint;
    obstaclePoint.position = wallPoint;
    obstaclePoint.time = scanTime;
    obstaclePoint.value = scan[index];
    obstaclePoint.angle = scanAngle;
    
    return obstaclePoint;
}

bool SonarBeamProcessing::isSegmentDone(estimator& estimator)
{
    std::list<obstaclePoint>::iterator endIt = estimator.segment.pointCloud.end();
    endIt--;
    switch (estimator.settings.segMode)
    {
        case forEachBeam:
            return true;
        case forEachEdge:
            if (estimator.segment.pointCloud.begin() == estimator.segment.latestBeam || 
                endIt == estimator.segment.latestBeam)
            {
                return true;
            }
        case noSegments:
        default:
            return false;
    }
}

void SonarBeamProcessing::updateSonarData(const base::samples::SonarScan& sonarScan)
{
    std::vector<base::samples::SonarScan::uint8_t> scan = sonarScan.scanData;
    double scanAngle = sonarScan.angle;
    double time_beetween_bins = sonarScan.time_beetween_bins;
    
    //check if angle is required
    bool required = false;
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if (scanAngle > it->settings.startAngle && scanAngle < it->settings.endAngle)
        {
            required = true;
        }
    }
    if (!required)
        return;
    
    std::vector<int> indexList;
    // analyze index
    if (enableThreshold && (time_beetween_bins * sonicVelocityInWater > 0))
    {
        //cut scan index
        int minIndex = (minThreshold * 2 * 10) / (time_beetween_bins * sonicVelocityInWater);
        int maxIndex = (maxThreshold * 2 * 10) / (time_beetween_bins * sonicVelocityInWater);
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
    
    //persist points and feed estimators
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if (scanAngle > it->settings.startAngle && scanAngle < it->settings.endAngle)
        {
            persistPoints(obstaclePoints, scanAngle, it->segment);
            it->segment.dirty = true;
        }
        if (it->segment.dirty && isSegmentDone(*it))
        {
            it->estimation->updateSegment(it->segment);
            it->segment.dirty = false;
        }
        
    }
    
}

}
