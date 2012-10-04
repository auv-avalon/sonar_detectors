#ifndef SONAR_FEATURE_EXTRACTION_HPP_
#define SONAR_FEATURE_EXTRACTION_HPP_

#include <vector>
#include <list>
#include <stdint.h>

#include "SonarDetectorTypes.hpp"
#include <base/samples/laser_scan.h>
#include <base/samples/sonar_beam.h>
#include <base/pose.h>
#include <machine_learning/GaussianParameters.hpp>

namespace sonar_detectors
{
    
struct FeatureCandidate
{
    int beam_index;
    float mean_value;
    float plain_value;
    double probability;
    FeatureCandidate() : 
        beam_index(-1), mean_value(0.0f), plain_value(0.0f), probability(1.0) {};
    bool operator<(const FeatureCandidate &fc) const { return (probability < fc.probability); }
};

struct HoughEntry
{
    unsigned beam_id;
    base::Vector2d feature2d;
    float probability;
    std::vector<base::Vector3d> intersection_points;
    HoughEntry() : beam_id(0), feature2d(Eigen::Vector2d::Zero()), probability(1.0) {};
};

class FeatureExtraction
{   
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
     * @return sorted feature candidates
     */
    std::vector<FeatureCandidate> computeDerivativeFeatureCandidates(const std::vector<float>& beam);
    /**
     * Configuration for the featureDerivativeHistory.
     * @param derivative_history_length num of the derivatives to build the history
     * @param feature_threshold threshold for new features in percent of the mean of the last |best_values_size| best features
     * @param best_values_size describes how many best-features will be saved to build the threshold
     * @param signal_balancing signal_balancing is a slope that corrects the signal moderation
     * @param plain_length the length of the plain in percent of signal length, if it is 0 the plain will be ignored
     * @param plain_threshold the threshold to be a acceptable plain in percent of the mean signal strength
     */
    void setDerivativeFeatureConfiguration(const unsigned int &derivative_history_length, const float &feature_threshold, const unsigned int &best_values_size, 
        const float &signal_balancing, const float &plain_length, const float &plain_threshold);
    /**
     * Provides intermediate results of getFeatureDerivativeHistory for debugging purposes.
     */
    void getDerivativeFeatureDebugData(std::vector<float> &minimum_derivative, float &value_threshold, float &plain_window_threshold);
    
    /**
     * Reinforces candidates on a line. Uses the hough transformation to reweight the candidates 
     * being on a line.
     * @param feature_candidates feature candidates sorted
     * @param bearing current bearing of the sonar
     * @param spatial_resolution spatial resolution in the current sonar beam
     * @param beam_size size of the current sonar beam
     */
    void enforceLines(std::vector<FeatureCandidate> &feature_candidates, const base::Angle &bearing, double spatial_resolution, unsigned beam_size);
    /**
     * Sets the configuration for enforceLines.
     * @param max_hough_history count of the last beams of which the candidates will be used
     * @param max_candidates_per_beam count of how many candidates per beam will be used
     * @param enforce_line_pos_rate weight of the impact of the line enforcement in percent
     * @param minimum_enforce_line_value probability threshold to be still a possible candidates
     * @param enforce_line_beam_covariance covariance for the 1d gaussian function which applies each estimated line on the actual sonarbeam
     */
    void setEnforceLinesConfiguration(unsigned int max_hough_history, unsigned int max_candidates_per_beam, double enforce_line_pos_rate, double minimum_enforce_line_value, double enforce_line_beam_covariance);
    /**
     * Debug data of the line enforcement process.
     */
    void getEnforceLinesDebugData(std::list<HoughEntry> &hough_entries, std::vector<base::Vector3d> &force_wall_pos);
    
    /**
     * Filters the candidates by a threshold over the last |average_length| candidates.
     * @param feature_candidates the feature candidates
     * @param probability_threshold times the center of the last |average_length| candidates is the threshold
     * @param average_length count of the candidates of which the center will be computed
     */
    void filterCandidates(std::vector<FeatureCandidate> &feature_candidates, double probability_threshold = 0.5, unsigned average_length = 100);
    
    
    /** ## Methods to create an obstaclePoint or a LaserScan of a given index in a beam ## **/
    
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
    base::Vector2d computeHoughSpaceIntersection(base::Vector2d &feature1, base::Vector2d &feature2);

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
    
    // enforce lines config
    MATRIX_XD(2) hough_covariance;
    MATRIX_XD(1) beam_covariance;
    double enforce_line_pos_rate;
    double minimum_enforce_line_value;
    unsigned int max_hough_history;
    unsigned int max_candidates_per_beam;
    
    unsigned int minimumIndex;
    double minimumValue;
    
private:
    std::vector<float> min_derivative;
    
    unsigned hough_group_id;
    std::list<HoughEntry> hough_entries;
    std::vector<base::Vector3d> force_wall_pos;
    
    std::vector<double> probability_treshold_history;
    double probability_treshold_sum;
    unsigned probability_treshold_cooldown;
    double probability_history_treshold;
    base::Angle last_angle;
};

}

#endif
