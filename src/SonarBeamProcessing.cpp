#include "SonarBeamProcessing.hpp"
#include <iostream>

namespace avalon
{
SonarBeamProcessing::SonarBeamProcessing(BeamMode beamMode)
{
    this->beamMode = beamMode;
    minThreshold = 0;
    maxThreshold = 100;
    enableThreshold = false;
    minResponseValue = 0;
    position.setZero();
    orientation = Eigen::Quaterniond::Identity();
    indexWindowSize = 25;
}

SonarBeamProcessing::~SonarBeamProcessing()
{
    
}

void SonarBeamProcessing::selectBeamMode(BeamMode mode)
{
    beamMode = mode;
}

void SonarBeamProcessing::updateOrientation(const base::Orientation& orientation)
{
    this->orientation = orientation;
}

void SonarBeamProcessing::updatePosition(const base::Position& position)
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

std::vector<int> SonarBeamProcessing::computeSonarScanIndex(const std::vector<base::samples::SonarScan::uint8_t>& scan, int minIndex, int maxIndex, int minValue)
{
    std::vector<int> indexList;
    int index = -1;
    int nextIndex = 0;

    switch (beamMode)
    {
        //sliding window approach 
        case globalMaximum:
        {
            if(scan.size() < maxIndex)
                maxIndex = scan.size();
            
            while(scan[minIndex] > 5 && minIndex < maxIndex)
                minIndex++;

            unsigned int act_window_value = 0;
            unsigned int best_window_value = 0;
            unsigned int best_window_pos = minIndex;

            //fill window
            unsigned end_index = minIndex + indexWindowSize;
            if(end_index > scan.size())
                    end_index = scan.size();
            for(int i = minIndex; i < end_index ; ++i)
                act_window_value += scan[i];

            //slide window 
            for(int i = end_index; i < maxIndex; i++)
            {
                if(act_window_value > best_window_value)
                {
                    best_window_value = act_window_value;
                    best_window_pos = i - indexWindowSize;
                }
                act_window_value+= scan[i];
                act_window_value-= scan[i-indexWindowSize];
            }
            unsigned int best_val = 0;
            int best_index = -1;
            end_index = best_window_pos + indexWindowSize;

            //find maximum insight the best window
            for(int i = best_window_pos; i < end_index; ++i)
            {
                if(scan[i]> best_val)
                {
                    best_val = scan[i];
                    best_index = i;
                }
            }

            if (best_index >= 0 && scan[best_index] > minValue)
                indexList.push_back(best_index);
            break;
        }
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

obstaclePoint SonarBeamProcessing::computeObstaclePoint(const int& index, const base::samples::SonarScan& sonarScan, const base::Orientation& orientation)
{
    double scanAngle = sonarScan.angle;
    double time_beetween_bins = sonarScan.time_beetween_bins;
    double distance = (double)index * sonarScan.getScale(sonicVelocityInWater);
        
    Eigen::Vector3d wallPoint(-distance,0,0);
    Eigen::Vector3d topPoint(0,0,1);
    
    wallPoint = orientation * wallPoint;
    topPoint = orientation * topPoint;
    
    Eigen::AngleAxisd rotate(-scanAngle,topPoint);
    
    wallPoint = rotate * wallPoint;
        
    avalon::obstaclePoint obstaclePoint;
    obstaclePoint.position = wallPoint;
    obstaclePoint.time = sonarScan.time;
    obstaclePoint.value = sonarScan.scanData[index];
    obstaclePoint.angle = scanAngle;
    
    return obstaclePoint;
}

void SonarBeamProcessing::updateSonarData(const base::samples::SonarScan& sonarScan)
{
    double scanAngle = sonarScan.angle;
    double time_beetween_bins = sonarScan.time_beetween_bins;
    
    //check if angle is required
    bool required = false;
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        if (it->settings.startAngle < 0 || it->settings.endAngle < 0 || scanAngle > it->settings.startAngle && scanAngle < it->settings.endAngle)
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
        int minIndex = (minThreshold * 2) / (time_beetween_bins * sonicVelocityInWater);
        int maxIndex = (maxThreshold * 2) / (time_beetween_bins * sonicVelocityInWater);
        indexList = computeSonarScanIndex(sonarScan.scanData, minIndex, maxIndex, minResponseValue);
    }
    else
    {
        indexList = computeSonarScanIndex(sonarScan.scanData, 0, sonarScan.scanData.size(), minResponseValue); 
    }
    
    std::vector<obstaclePoint> obstaclePoints;
    if (!indexList.empty())
    {
        // calculate points
        while (indexList.size() > 0)
        {
            avalon::obstaclePoint obstaclePoint = computeObstaclePoint(indexList.back(), sonarScan, orientation);
            indexList.pop_back();
            obstaclePoints.push_back(obstaclePoint);
        }
    }
    
    //feed estimators
    for(std::vector<estimator>::iterator it = estimators.begin(); it != estimators.end(); it++)
    {
        // send new features to the estimators if they are in their scan range
        if (it->settings.startAngle < 0 || it->settings.endAngle < 0
            || scanAngle > it->settings.startAngle && scanAngle < it->settings.endAngle)
        {
            it->estimation->updateSegment(obstaclePoints);
        }
    }
    
}

}
