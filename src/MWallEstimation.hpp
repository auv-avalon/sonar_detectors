#ifndef M_WALL_ESTIMATION_HPP_
#define M_WALL_ESTIMATION_HPP_

#include "SonarEstimation.hpp"
#include "SonarMap.hpp"

namespace sonar_detectors
{
    /**
     * This class tries to estimate a line on a given pointcloud.
     * Tries to find only a wall under strong requirements with 
     * the goal to provide only true wall positions.
     */
    class MWallEstimation : public SonarEstimation
    {
    public:
        MWallEstimation();
        ~MWallEstimation();
        const std::pair<base::Vector3d, base::Vector3d>& getWall() const;
        const std::vector<base::Vector3d>& getPointCloud();
        std::vector<base::Vector3d> getCompletePointCloud();
        void setParameters(double line_fit_threshold, double min_fit_rate, double dbscan_epsilon, double angular_resolution);
        
    protected:
        virtual void updateFeatureIntern(const base::samples::LaserScan& feature);
        
    private:
        struct wallCandidate
        {
            std::vector<base::Vector3d> point_cloud;
            std::pair<base::Vector3d, base::Vector3d> line;
            double line_distance;
            double fit_rate;
        };
        
    private:
        sonar_detectors::SonarMap< base::Vector3d > sonarMap;
        std::pair<base::Vector3d, base::Vector3d> wall;
        std::vector<base::Vector3d> pointCloud;
        unsigned int min_count_pointcloud;
        double min_wall_distance;
        double line_fit_threshold;
        double min_fit_rate;
        double dbscan_epsilon;
        double angular_resolution;
    };
}

#endif