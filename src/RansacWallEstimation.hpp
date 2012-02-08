#ifndef RANSAC_WALL_ESTIMATION_HPP_
#define RANSAC_WALL_ESTIMATION_HPP_

#include "SonarEstimation.hpp"
#include "SonarMap.hpp"

namespace sonar_detectors
{
    /**
     * This class trys to estimate a line on a given pointcloud.
     * 
     * estimationSettings should be:
     * - angle range less than PI
     */
    class RansacWallEstimation : public SonarEstimation
    {
    public:
        RansacWallEstimation();
        ~RansacWallEstimation();
        const std::pair<base::Vector3d, base::Vector3d>& getWall() const;
        const std::vector<base::Vector3d>& getPointCloud();
        void setRansacParameters(double ransac_threshold, double ransac_fit_rate, double dbscan_epsilon);
        
    protected:
        virtual void updateFeatureIntern(const base::samples::LaserScan& feature);
        
    private:
        sonar_detectors::SonarMap< base::Vector3d > sonarMap;
        std::pair<base::Vector3d, base::Vector3d> wall;
        std::vector<base::Vector3d> pointCloud;
        unsigned int min_count_pointcloud;
        double ransac_threshold;
        double ransac_fit_rate;
        double dbscan_epsilon;
    };
}

#endif