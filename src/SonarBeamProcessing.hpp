#include <base/samples/sonar_scan.h>
#include <base/eigen.h>
#include <base/samples/rigid_body_state.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <list>
#include <vector>
#include <cmath>
#include <avalonmath.h>

#include <opencv/cv.h>

namespace avalon
{
    
enum BeamMode 
{
    globalMaximum,
    firstLokalMaximum,
    allLokalMaxima,
    allEntries
};

enum PersistMode 
{
    persistNewScans,
    persistAll
};

static const double sonicVelocityInWater = 1500.0;
static const double newSegmentThreshold = M_PI / 8;
    
struct obstaclePoint
{
    base::Position position;
    base::samples::SonarScan::uint8_t value;
    base::Time time;
    double angle;
    obstaclePoint()
    : position(0,0,0), value(0), angle(0){}
};

struct scanSegment
{
    std::list<obstaclePoint> pointCloud;
    std::list<obstaclePoint>::iterator latestBeam;
    std::vector< std::pair<base::Vector3d, base::Vector3d> > walls;
    base::Vector3d virtualpoint;
};
  
class SonarBeamProcessing 
{   
    public:
        SonarBeamProcessing(BeamMode beamMode, PersistMode persistMode);
        ~SonarBeamProcessing();
        void updateSonarData(const base::samples::SonarScan& sonarScan);
        void updateOrientation(base::Orientation& orientation);
        void updatePosition(base::Position& position);
        void selectBeamMode(BeamMode mode);
        void selectPersistMode(PersistMode mode);
        void enableBeamThreshold(bool b);
        void setBeamThreshold(double minThreshold, double maxThreshold);
        void enableWallEstimation(bool b);
        void setMinResponseValue(unsigned minValue);
        base::Vector3d getVirtualPoint() const;
        
        const std::vector<scanSegment>* getPoints() const;
        
    private:
        int inputCount;
        std::vector<scanSegment> segments;
        base::Orientation orientation;
        base::Position position;
        BeamMode beamMode;
        PersistMode persistMode;
        bool enableThreshold;
        double minThreshold;
        double maxThreshold;
        bool estimateWalls;
        unsigned minResponseValue;
        bool newOrientationOrPosition;
        std::vector<int> computeSonarScanIndex(const std::vector<base::samples::SonarScan::uint8_t>& scan, const unsigned& minIndex, const unsigned& maxIndex, const unsigned& minValue);
        int getNextMaximum(int startIndex, int endIndex, unsigned minValue, const std::vector<base::samples::SonarScan::uint8_t>& scan);
        avalon::obstaclePoint computeObstaclePoint(const int& index, const base::samples::SonarScan& sonarScan);
        void persistPoints(const std::vector<obstaclePoint>& obstaclePoints, double& angle, scanSegment& segment);
        void createNewSegment();
        void computeWall(avalon::scanSegment& segment);
        void computeVirtualPoint(avalon::scanSegment& segment);
};

}
