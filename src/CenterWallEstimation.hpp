#ifndef CENTER_WALL_ESTIMATION_HPP_
#define CENTER_WALL_ESTIMATION_HPP_

#include "SonarEstimation.hpp"
#include <map>

namespace sonar_detectors
{
    /**
     * This class tries to estimate a line on a given pointcloud.
     * Builds the center point of the left and the right range of the sonar range.
     * The line between those points is the most likely wall.
     */
    class CenterWallEstimation : public SonarEstimation
    {
    public:
        CenterWallEstimation();
        ~CenterWallEstimation();
        
        void setMinScanPoints(unsigned int min_scan_points);
        void setFadingOutFactor(double factor);
        void setSupposedWallAngle(const base::Angle &supposed_wall_angle);
        void setWallAngleVariance(double angular_range);
        const std::pair<base::Vector3d, base::Vector3d> getWall() const;
        std::vector<base::Vector3d> getPointCloud() const;
        
    protected:
        virtual void updateFeatureIntern(const base::samples::LaserScan& feature);
        void getSubPointsFromMap(std::vector<base::Vector3d> &points, const base::Angle &start_angle, const base::Angle &end_angle);
        base::Vector3d calcCenterOfGeometry(const std::vector<base::Vector3d> &points);
        void getSubPointCount(unsigned int &count, const base::Angle &start_angle, const base::Angle &end_angle);
        
    protected:
        std::multimap<base::Angle, base::Vector3d> feature_map;
        unsigned int min_scan_points; //Min number of valid scan points for valid wall 
        double fading_out_factor;
        base::Vector3d left_center;
        base::Vector3d right_center;
        base::Angle supposed_wall_angle;
        double angular_range_variance;
        double applied_variance;
        std::pair<base::Vector3d, base::Vector3d> wall;
    };
}

#endif