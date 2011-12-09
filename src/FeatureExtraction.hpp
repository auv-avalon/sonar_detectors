#ifndef SONAR_FEATURE_EXTRACTION_HPP_
#define SONAR_FEATURE_EXTRACTION_HPP_

#include <vector>
#include <list>
#include <stdint.h>

namespace sonar_detectors
{

class FeatureExtraction
{
public:
    FeatureExtraction();
    ~FeatureExtraction();
    
    int getFeatureGlobalMaxima(const std::vector<float>& beam, const unsigned int &indexWindowSize = 20);
    int getFeatureMaximalLevelDifference(const std::vector<float>& beam, const unsigned int &indexWindowSize = 20);
    int getFeatureDerivativeHistory(const std::vector<float>& beam, const unsigned int &history_length, const float &threshold, const bool &indoor_mode = false);
    
    std::vector<float> convertBeam(const std::vector<uint8_t>& beam);
    void setBoundingBox(const double radius, const double sampling_interval, const int speed_of_sound = 1500);
    void setMinResponseValue(const double minValue);
    
protected:
    void addToDerivativeHistory(const std::vector<float>& beam, const unsigned int &history_length);

protected:
    std::list< std::vector<float>* > derivativeHistory;
    unsigned int minimumIndex;
    double minimumValue;
};

}

#endif
