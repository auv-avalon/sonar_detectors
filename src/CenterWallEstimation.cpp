#include "CenterWallEstimation.hpp"
#include "SonarDetectorMath.hpp"
#include <iostream>

namespace sonar_detectors
{
    
const unsigned int MIN_SCAN_POINTS = 7;    //Min number of valid scan points for valid wall 
    
CenterWallEstimation::CenterWallEstimation() : 
                      fading_out_factor(0.02)
{
    wall.first = base::Vector3d(0.0,0.0,0.0);
    wall.second = base::Vector3d(0.0,0.0,0.0);
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
        
        base::Angle global_mid_angle = global_start_angle - (range * 0.5);
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
    
    if(end_angle.rad - start_angle.rad > M_PI)
        range_switch = true;
    
    for(std::multimap<base::Angle, base::Vector3d>::iterator it = feature_map.begin(); it != feature_map.end(); it++)
    {
        if((range_switch && (it->first < start_angle || it->first > end_angle)) ||
           (!range_switch && ( it->first < start_angle && it->first > end_angle)))
            points.push_back(it->second);
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