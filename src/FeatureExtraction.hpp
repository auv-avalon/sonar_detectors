#ifndef SONAR_FEATURE_EXTRACTION_HPP_
#define SONAR_FEATURE_EXTRACTION_HPP_

#include <vector>
#include <list>
#include <stdint.h>

#include "SonarDetectorTypes.hpp"
#include <base/samples/laser_scan.h>
#include <base/samples/sonar_beam.h>
#include <base/pose.h>

namespace sonar_detectors
{

class FeatureExtraction
{   
public:
    struct FeatureCandidates
    {
        int beam_index;
        float mean_value;
        float plain_value;
        FeatureCandidates() : 
            beam_index(-1), mean_value(0.0f), plain_value(0.0f) {};
    };
    
public:
    FeatureExtraction();
    ~FeatureExtraction();
    
    /** ## Old feature detectors. Use 'setBoundingBox' and 'setMinResponseValue' for configuration ## **/
    
    /**
     * Detects the global maxima in a signal. First the maximum sliding window, 
     * than the maximum peak in that window.
     * @param beam the sonar signal
     * @param indexWindowSize size of the sliding window
     * @return index of the peak
     */
    int getFeatureGlobalMaxima(const std::vector<float>& beam, const unsigned int &indexWindowSize = 20);
    /**
     * Detects the maximum negative difference between two sliding windows.
     * @param beam the sonar signal
     * @param indexWindowSize size of the sliding windows
     * @return index of the peak
     */
    int getFeatureMaximalLevelDifference(const std::vector<float>& beam, const unsigned int &indexWindowSize = 20);
    /**
     * Sets a radial bounding box.
     * @param radius of the bounding box
     * @param sampling_interval of the sonar beam
     * @param speed_of_sound in water
     */
    void setBoundingBox(const double radius, const double sampling_interval, const int speed_of_sound = 1500);
    /**
     * Sets a threshold for the sonar signals.
     * @param minValue threshold
     */
    void setMinResponseValue(const double minValue);
    
    
    /** ## New feature detector. Use 'featureDerivativeHistoryConfiguration' for configuration ## **/
    
    /**
     * Returns the spot of the maximum accumulation in the history-derivative. 
     * The history-derivative is the minimum of the last |derivative_history_length| derivatives.
     * @param beam the sonar signal
     * @return index of the peak
     */
    int getFeatureDerivativeHistory(const std::vector<float>& beam);
    /**
     * Configuration for the featureDerivativeHistory.
     * @param derivative_history_length num of the derivatives to build the history
     * @param feature_threshold threshold for new features in percent of the mean of the last |best_values_size| best features
     * @param best_values_size describes how many best-features will be saved to build the threshold
     * @param signal_balancing signal_balancing is a slope that corrects the signal moderation
     * @param plain_length the length of the plain in percent of signal length, if it is 0 the plain will be ignored
     * @param plain_threshold the threshold to be a acceptable plain in percent of the mean signal strength
     */
    void featureDerivativeHistoryConfiguration(const unsigned int &derivative_history_length, const float &feature_threshold, const unsigned int &best_values_size, 
        const float &signal_balancing, const float &plain_length, const float &plain_threshold);
    /**
     * Provides intermediate results of getFeatureDerivativeHistory for debugging purposes.
     */
    void getFDHDebugData(std::vector<float> &minimum_derivative, float &value_threshold, float &plain_window_threshold, std::vector<FeatureCandidates> &candidates);
    
    
    /**
     * Computes an obstaclePoint from a given SonarBeam and index.
     * If the orientation is not the identity the resulting vector is absolut in the orientation. 
     * @param index of the feature in the beam
     * @param sonarBeam the sonar beam
     * @param orientation orientation
     * @returns sonar_detectors::obstaclePoint
     */
    static sonar_detectors::obstaclePoint computeObstaclePoint(const int& index, const base::samples::SonarBeam& sonar_beam, const base::Orientation& orientation = base::Quaterniond::Identity());
    /**
     * Computes an LaserScan from a given SonarBeam and index. The LaserScan
     * has ony one entry, which is the feature.
     * @param index of the feature in the beam
     * @param sonarBeam the sonar beam
     * @param orientation orientation
     * @returns a base::samples::LaserScan
     */
    static base::samples::LaserScan computeLaserScan(const int& index, const base::samples::SonarBeam& sonar_beam);
    
    
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
    float plain_window_threshold;
    
    unsigned int minimumIndex;
    double minimumValue;
    
private:
    std::vector<float> min_derivative;
    std::vector<FeatureCandidates> feature_candidates;
};

}

#endif
