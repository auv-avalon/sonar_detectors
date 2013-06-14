#ifndef SONAR_DEPTH_MAP
#define SONAR_DEPTH_MAP

#include <vector>
#include <base/eigen.h>

namespace sonar_detectors
{

/* This class handles clustering of cartesian coordinate points. It acts
 * as an interface to the clustering library.
 */
class SonarDepthMap
{   
    public:
        SonarDepthMap();
        ~SonarDepthMap();
	
	 double resolution;
	 double minY;
	 double maxY;
	 double minX;
	 double maxX;
	
	 std::vector< std::vector<base::Vector3d> > depthPoints;
	 
        
    private:
        
};

}
#endif