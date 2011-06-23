#ifndef WALL_ESTIMATION_HPP_
#define WALL_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"

namespace avalon
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
        const base::Vector3d getRelativeVirtualPoint();
        void setRansacParameters(double threshold, double fit_rate);
        virtual void updateSegment(const avalon::scanSegment& segment);
        
    private:
        void computeVirtualPoint();
        
        std::vector< std::pair<base::Vector3d, base::Vector3d> > walls;
        base::Vector3d virtualpoint;
        int min_count_pointcloud;
        double ransac_threshold;
        double ransac_fit_rate;
    };
}

#endif