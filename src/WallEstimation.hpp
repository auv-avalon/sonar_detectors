#ifndef WALL_ESTIMATION_HPP_
#define WALL_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include "SonarEstimation.hpp"

namespace avalon
{
    class WallEstimation : public SonarEstimation
    {
    public:
        WallEstimation();
        ~WallEstimation();
        const std::vector< std::pair<base::Vector3d, base::Vector3d> > getWalls() const;
        const base::Vector3d getVirtualPoint();
        const base::Vector3d getRelativeVirtualPoint();
        virtual void updateSegment(const avalon::scanSegment& segment);
        
    private:
        void computeWall(const std::list<obstaclePoint>& pointCloud, std::list<obstaclePoint>::const_iterator latestBeam);
        void computeVirtualPoint();
        
        std::vector< std::pair<base::Vector3d, base::Vector3d> > walls;
        base::Vector3d virtualpoint;
        
    };
}

#endif