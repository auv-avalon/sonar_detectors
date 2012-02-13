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
        
        static void clusterPointCloud(std::list<base::Vector3d*> *pointCloud, 
                                                                std::vector< std::set<base::Vector3d*> >& cluster,
                                                                unsigned int min_pts, 
                                                                double epsilon, 
                                                                bool use_z = false,
                                                                bool use_dynamic_epsilon = false,
                                                                double dynamic_epsilon_weight = 1.0);
        
    private:
        
};

}

#endif
