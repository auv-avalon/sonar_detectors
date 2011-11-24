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
    int getFeatureMaximalLevelDifference(const std::vector<float>& beam);
    
    std::vector<float> convertBeam(const std::vector<uint8_t>& beam);
    
    void setBoundingBox(const double radius, const double sampling_interval, const int speed_of_sound = 1500);
    void setMinResponseValue(const double minValue);
    

private:
    unsigned int minimumIndex;
    double minimumValue;
    int indexWindowSize;
};
    
}

#endif