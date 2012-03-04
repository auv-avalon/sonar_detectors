#include "CenterWallEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include <iostream>

namespace sonar_detectors
{
    
const unsigned int MIN_SCAN_POINTS = 7;    //Min number of valid scan points for valid wall 
    
CenterWallEstimation::CenterWallEstimation() : 
                      fading_out_factor(0.02),
                      applied_variance(0.0),
                      angular_range_variance(0.0)
{
    wall.first = base::Vector3d(0.0,0.0,0.0);
    wall.second = base::Vector3d(0.0,0.0,0.0);
    supposed_wall_angle.rad = 0.0;
}

CenterWallEstimation::~CenterWallEstimation()
{

}

void CenterWallEstimation::updateFeatureIntern(const base::samples::LaserScan& feature)
{
    std::vector<Eigen::Vector3d> featureVector;
    feature.convertScanToPointCloud(featureVector);
    base::Angle range = global_start_angle - global_end_angle;
    if(range.rad == 0)
    {
        std::cerr << "CenterWallEstimation: estimation angle is zero!" << std::endl;
        return;
    }
    
    // fading out points
    double fading_out_factor = this->fading_out_factor * (M_PI_2 / range.getRad());
    for(std::multimap<base::Angle, base::Vector3d>::iterator it = feature_map.begin(); it != feature_map.end(); it++)
    {
        it->second.z() -= fading_out_factor;
        if(it->second.z() <= 0.0)
            feature_map.erase(it);
    }
    
    if(featureVector.size() > 0)
    {
        // add new features
        for(std::vector<Eigen::Vector3d>::iterator it = featureVector.begin(); it != featureVector.end(); it++)
        {
            // TODO use the remission value here in some rate
            it->z() = 1.0;
            feature_map.insert(std::make_pair<base::Angle, Eigen::Vector3d>(base::Angle::fromRad(feature.start_angle), *it));
        }
        
        // seperate scan points into left and right
        std::vector<base::Vector3d> left_points;
        std::vector<base::Vector3d> right_points;
        
        base::Angle global_mid_angle = global_heading + supposed_wall_angle + base::Angle::fromRad(applied_variance);
        // use a angular range variance to better detect corners 
        if(angular_range_variance != 0.0)
        {
            unsigned int count_left, count_right;
            getSubPointCount(count_left, global_start_angle, global_mid_angle);
            getSubPointCount(count_right, global_mid_angle, global_end_angle);
            if(angular_range_variance > 0.0)
            {
                // left variance
                if(count_left > count_right * 2.0 && applied_variance < angular_range_variance)
                {
                    applied_variance += angular_range_variance * 0.05;
                    if(applied_variance > angular_range_variance)
                        applied_variance = angular_range_variance;
                }
                else if(applied_variance > 0.0 && count_left < count_right * 1.5)
                {
                    applied_variance -= angular_range_variance * 0.05;
                    if(applied_variance < 0.0)
                        applied_variance = 0.0;
                }
            }
            else
            {
                // right variance
               if(count_right > count_left * 2.0 && applied_variance > angular_range_variance)
                {
                    applied_variance += angular_range_variance * 0.05;
                    if(applied_variance < angular_range_variance)
                        applied_variance = angular_range_variance;
                }
                else if(applied_variance < 0.0 && count_right < count_left * 1.5)
                {
                    applied_variance -= angular_range_variance * 0.05;
                    if(applied_variance > 0.0)
                        applied_variance = 0.0;
                }
            }
            global_mid_angle = global_heading + supposed_wall_angle + base::Angle::fromRad(applied_variance);
        }
        
        getSubPointsFromMap(left_points, global_start_angle, global_mid_angle);
        getSubPointsFromMap(right_points, global_mid_angle, global_end_angle);
        
        // compute wall position
        if(left_points.size() < MIN_SCAN_POINTS || right_points.size() < MIN_SCAN_POINTS)
        {
            wall.first = base::Vector3d(0.0,0.0,0.0);
            wall.second = base::Vector3d(0.0,0.0,0.0);
        }
        else
        {
            left_center = calcCenterOfGeometry(left_points);
            right_center = calcCenterOfGeometry(right_points);
            
            base::Vector3d support_vector = left_center;
            support_vector.z() = 0.0;
            base::Vector3d direction_vector = right_center - left_center;
            if(direction_vector.x() == 0.0 && direction_vector.y() == 0.0)
                direction_vector = right_center;
            direction_vector.z() = 0.0;
            wall.first = computIntersection(std::make_pair<base::Vector3d, base::Vector3d>(support_vector, direction_vector), base::Vector3d(0,0,0));
            wall.second = direction_vector;
        }
    }
}

void CenterWallEstimation::getSubPointsFromMap(std::vector< base::Vector3d >& points, const base::Angle& start_angle, const base::Angle& end_angle)
{
    bool range_switch = false;
    
    if(end_angle.rad - start_angle.rad > 0.0)
        range_switch = true;
    
    for(std::multimap<base::Angle, base::Vector3d>::iterator it = feature_map.begin(); it != feature_map.end(); it++)
    {
        if((range_switch && (it->first < start_angle || it->first > end_angle)) ||
           (!range_switch && ( it->first < start_angle && it->first > end_angle)))
            points.push_back(it->second);
    }
}

void CenterWallEstimation::getSubPointCount(unsigned int& count, const base::Angle& start_angle, const base::Angle& end_angle)
{
    bool range_switch = false;
    count = 0;
    
    if(end_angle.rad - start_angle.rad > 0.0)
        range_switch = true;
    
    for(std::multimap<base::Angle, base::Vector3d>::iterator it = feature_map.begin(); it != feature_map.end(); it++)
    {
        if((range_switch && (it->first < start_angle || it->first > end_angle)) ||
           (!range_switch && ( it->first < start_angle && it->first > end_angle)))
            count++;
    }
}

base::Vector3d CenterWallEstimation::calcCenterOfGeometry(const std::vector<base::Vector3d> &points)
{
    if(points.empty())
        return base::Vector3d(0.0,0.0,0.0);

    float value_x = 0;
    float value_y = 0;
    float total_strength = 0;
    for(std::vector<base::Vector3d>::const_iterator iter = points.begin(); iter != points.end(); iter++)
    {
        value_x += iter->x() * iter->z();
        value_y += iter->y() * iter->z();
        total_strength += iter->z();
    }
    if(total_strength == 0)
        return base::Vector3d(0.0,0.0,0.0);

    return base::Vector3d(value_x/total_strength,value_y/total_strength,total_strength/points.size());
}

void CenterWallEstimation::setFadingOutFactor(double factor)
{
    fading_out_factor = factor;
}

void CenterWallEstimation::setSupposedWallAngle(base::Angle supposed_wall_angle)
{
    this->supposed_wall_angle = supposed_wall_angle;
}

void CenterWallEstimation::setWallAngleVariance(double angular_range)
{
    this->angular_range_variance = angular_range;
}

const std::pair< base::Vector3d, base::Vector3d > CenterWallEstimation::getWall() const
{
    return wall;
}

std::vector< base::Vector3d > CenterWallEstimation::getPointCloud() const
{
    std::vector<base::Vector3d> point_cloud;
    for(std::multimap<base::Angle, base::Vector3d>::const_iterator it = feature_map.begin(); it != feature_map.end(); it++)
    {
        point_cloud.push_back(it->second);
    }
    return point_cloud;
}

}