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
    int getFeatureDerivativeHistory(const std::vector<float>& beam);
    
    std::vector<float> convertBeam(const std::vector<uint8_t>& beam);
    void setBoundingBox(const double radius, const double sampling_interval, const int speed_of_sound = 1500);
    void setMinResponseValue(const double minValue);
    
protected:
    void addToDerivativeHistory(const std::vector<float>& beam, const unsigned int &history_length);

protected:
    std::list< std::vector<float>* > derivativeHistory;
    
    // feature extraction config
    unsigned int derivative_history_length;
    bool force_plain;
    float plain_length;
    float plain_threshold;
    float signal_balancing;
    float feature_threshold;
    int cooldown_threshold;
    unsigned int best_values_size;
    std::vector< float > best_values;
    float sum_best_values;
    float value_threshold;
    
    unsigned int minimumIndex;
    double minimumValue;
};

}

#endif
