#ifndef WALL_ESTIMATION_HPP_
#define WALL_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"
#include "SonarMap.hpp"

namespace sonar_detectors
{
    /**
     * This class trys to estimate a line on a given pointcloud.
     * 
     * estimationSettings should be:
     * - angle range less than PI
     * - SegmentMode: forEachEdge
     */
    class WallEstimation : public SonarEstimation
    {
    public:
        WallEstimation();
        ~WallEstimation();
        const std::vector< std::pair<base::Vector3d, base::Vector3d> > getWalls() const;
        const base::Vector3d getVirtualPoint();
        std::list<base::Vector3d> getPointCloud();
        void setRansacParameters(double threshold, double fit_rate);
        
    protected:
        virtual void updateFeatureIntern(const base::samples::LaserScan& feature);
        
    private:
        void computeVirtualPoint();
        
        sonar_detectors::SonarMap< base::Vector3d > sonarMap;
        std::list< base::Vector3d >* featureList;
        std::vector< std::pair<base::Vector3d, base::Vector3d> > walls;
        base::Vector3d virtualpoint;
        unsigned int min_count_pointcloud;
        double ransac_threshold;
        double ransac_fit_rate;
    };
}

#endif