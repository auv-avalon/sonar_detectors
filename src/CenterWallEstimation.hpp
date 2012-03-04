#ifndef CENTER_WALL_ESTIMATION_HPP_
#define CENTER_WALL_ESTIMATION_HPP_

#include "SonarEstimation.hpp"
#include <map>

namespace sonar_detectors
{
    /**
     * This class trys to estimate a line on a given pointcloud.
     * 
     * estimationSettings should be:
     * - angle range less than PI
     */
    class CenterWallEstimation : public SonarEstimation
    {
    public:
        CenterWallEstimation();
        ~CenterWallEstimation();
        
        void setFadingOutFactor(double factor);
        void setSupposedWallAngle(base::Angle supposed_wall_angle);
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