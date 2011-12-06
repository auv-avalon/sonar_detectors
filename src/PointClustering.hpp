#ifndef POINT_CLUSTERING
#define POINT_CLUSTERING

#include <machine_learning/DBScan.hpp>
#include <set>
#include <vector>
#include <boost/foreach.hpp>

namespace sonar_detectors
{

/* This class handles clustering of cartesian coordinate points. It acts
 * as an interface to the clustering library.
 */
class PointClustering
{   
    public:
        PointClustering();
        ~PointClustering();
        
        std::vector< std::set<base::Vector3d*> > clusterPointCloud(std::list<base::Vector3d*> *pointCloud, 
                                                                unsigned int min_pts, 
                                                                double epsilon, 
                                                                bool use_z = false);
        
    private:
        
};

}

#endif
