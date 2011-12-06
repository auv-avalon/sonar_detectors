#include "PointClustering.hpp"
#include <map>
#include <iostream>
#include <cassert>

namespace sonar_detectors
{

PointClustering::PointClustering()
{
    
}

PointClustering::~PointClustering()
{
    
}

std::vector< std::set<base::Vector3d*> > PointClustering::clusterPointCloud(std::list<base::Vector3d*> *pointCloud, 
                                                                        unsigned int min_pts, 
                                                                        double epsilon, 
                                                                        bool use_z)
{
    /* Get cluster result */
    machine_learning::DBScan dbs(pointCloud, min_pts, epsilon, use_z);
    std::map<base::Vector3d*, int> scan_result = dbs.scan();
    
    /* Convert result map to set vector */
    int cluster_count = dbs.getClusterCount();
    std::vector< std::set<base::Vector3d*> > clusterVec(cluster_count);
    
    //std::cout << "set sizes before" << std::endl;
    //BOOST_FOREACH(std::set<base::Vector3d*> set, clusterVec) {
    //    std::cout << set.size() << std::endl;
    //}
    
    std::map<base::Vector3d*, int>::iterator mapIt;
    for(mapIt = scan_result.begin(); mapIt != scan_result.end(); mapIt++) {
        int id = mapIt->second;
        //std::cout << "map value " << mapIt->second << " for point " << machine_learning::pointToString(*mapIt->first) << std::endl;
        // Ignore noise and unclassified points, add the rest to the sets
        if(id >= 0) {
            std::pair<std::set<base::Vector3d*>::iterator,bool> ret = clusterVec[id].insert(mapIt->first);
            assert(ret.second);
        }
    }
    //std::cout << "set sizes after" << std::endl;
    //BOOST_FOREACH(std::set<base::Vector3d*> set, clusterVec) {
    //    std::cout << set.size() << std::endl;
    //}
    
    return clusterVec;
}

} // namespace
