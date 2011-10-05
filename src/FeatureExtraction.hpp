#ifndef SONAR_FEATURE_EXTRACTION_HPP_
#define SONAR_FEATURE_EXTRACTION_HPP_

#include <vector>
#include <stdint.h>

namespace sonar_detectors
{

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