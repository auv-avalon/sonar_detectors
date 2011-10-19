#ifndef SONAR_FEATURE_EXTRACTION_HPP_
#define SONAR_FEATURE_EXTRACTION_HPP_

#include <vector>
#include <stdint.h>
#include <math.h>
#include <base/eigen.h>
#include <map>

#define M_PI_8 (M_PI_4 * 0.5)

namespace sonar_detectors
{
    
class SonarEnvironmentModel
{
public:
    SonarEnvironmentModel();
    
    void getExpectedObstaclePositions(const double beam_angle, int &pos_auv, int &pos_ground, int &pos_surface);
    
    void setAUVOrientation(const base::Quaterniond orientation);
    void setDistanceToGround(const double distance);
    void setDistanceToSurface(const double distance);
    void setSonarBeamProperties(const double sampling_interval, const float beamwidth_vertical = (35.0/180.0)*M_PI, const float speed_of_sound = 1500);
    
private:
    int getExpectedObstaclePosition(const double beamwidth_vertical_angle, const double distance_to_plain);
    int getAUVModelConstrains(const double beam_angle, double &beamwidth_vertical_down);
    
    std::vector<base::Vector2d> avalon_model;
    std::map<double, double> angle_to_edge_distance;
    
    double distance_to_ground;
    double distance_to_surface;
    double distance_to_avalon_model;
    
    base::Quaterniond orientation;
    double sampling_interval;
    float beamwidth_vertical;
    float speed_of_sound;
    
};

class FeatureExtraction
{
public:
    FeatureExtraction();
    
    int getFeatureGlobalMaxima(const std::vector<float>& beam);
    int getFeatureHighestWaveFromBehind(const std::vector<float>& beam);
    int getFeatureMaximalLevelDifference(const std::vector<float>& beam);
    
    std::vector<float> noFilter(const std::vector<uint8_t>& beam);
    std::vector<float> smoothFilter(const std::vector<uint8_t>& beam);
    std::vector<float> balancePointFilter(const std::vector<uint8_t>& beam);
    
    void removeInfluence(std::vector<float>& beam);
    
    void setBoundingBox(const double radius, const double sampling_interval, const int sonicVelocityInWater = 1500);
    void setMinResponseValue(double minValue);
    
private:
    unsigned int minimumIndex;
    double minimumValue;
    int indexWindowSize;
};
    
}

#endif