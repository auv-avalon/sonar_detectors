#include "FeatureExtraction.hpp"
#include <dsp_acoustics/FIRFilter.h>
#include <iostream>
#include <sonar_detectors/SonarEnvironmentModel.hpp>

namespace sonar_detectors
{
    
FeatureExtraction::FeatureExtraction() :
                    cooldown_threshold(0),
                    sum_best_values(0.0f),
                    value_threshold(0.0f),
                    plain_window_threshold(0.0f),
                    minimumIndex(0),
                    minimumValue(20.0),
                    hough_group_id(0),
                    probability_history_treshold(-1.0),
                    probability_treshold_sum(0.0),
                    probability_treshold_cooldown(0)
{
    // feature extraction config
    /* history length of the derivatives */
    derivative_history_length = 3;
    /* forces the feature to have a nearby empty plain behind falling derivative */
    force_plain = true;
        /* the length of the plain in percent of signal length */
        plain_length = 0.1f; //[0..1]
        /* the threshold to be a acceptable plain in percent of the mean signal strength */
        plain_threshold = 0.9f; //[0..1]
    /* signal_balancing is a slope that corrects the signal moderation. 
     * From 0.8 to 1.0 on the hole signal in case of signal_balancing = 0.2 */
    signal_balancing = 0.2f; //[0..1]
    /* describes how many best-features will be saved to build the threshold */
    best_values_size = 100;
    /* threshold for new features in percent of the mean of the last |best_values_size| best features */
    feature_threshold = 0.6f; //[0..1]
    
    
    hough_covariance << 0.628318531, 0.0, 0.0, 0.628318531; //0.1 * 2PI -- |
    
    beam_covariance << 2.0; // in m
    
    enforce_line_pos_rate = 0.5;
    
    max_hough_history = 5;
    
    max_candidates_per_beam = 4;
}

FeatureExtraction::~FeatureExtraction()
{
    for(std::list< std::vector<float>* >::iterator it = derivativeHistory.begin(); it != derivativeHistory.end(); it++)
    {
        delete *it;
    }
}
    
int FeatureExtraction::getFeatureGlobalMaxima(const std::vector<float>& beam, const unsigned int &indexWindowSize)
{    
    // cut up noise
    while(beam[minimumIndex] > 5.0f && minimumIndex < beam.size())
        minimumIndex++;

    float act_window_value = 0.0f;
    float best_window_value = 0.0f;
    unsigned int best_window_pos = minimumIndex;

    //fill window
    unsigned end_index = minimumIndex + indexWindowSize;
    if(end_index >= beam.size())
            end_index = beam.size() - 1;
    for(unsigned int i = minimumIndex; i < end_index ; ++i)
        act_window_value += beam[i];

    //slide window 
    for(unsigned int i = end_index; i < beam.size(); i++)
    {
        if(act_window_value > best_window_value)
        {
            best_window_value = act_window_value;
            best_window_pos = i - indexWindowSize;
        }
        act_window_value+= beam[i];
        act_window_value-= beam[i-indexWindowSize];
    }
    unsigned int best_val = 0;
    int best_index = -1;
    end_index = best_window_pos + indexWindowSize;

    //find maximum insight the best window
    for(unsigned int i = best_window_pos; i < end_index; ++i)
    {
        if(beam[i]> best_val)
        {
            best_val = beam[i];
            best_index = (int)i;
        }
    }

    if (best_index >= 0 && beam[best_index] > minimumValue)
        return best_index;
    else
        return -1;
}

int FeatureExtraction::getFeatureMaximalLevelDifference(const std::vector< float >& beam, const unsigned int &indexWindowSize)
{
    if(beam.size() == 0)
        return -1;
    
    float first_window_value = 0.0f;
    float second_window_value = 0.0f;
    float best_window_difference = -10000.0f;
    float best_value_difference = 0.0f;
    unsigned int best_window_pos = 0;
    
    //fill windows
    unsigned first_window_index = beam.size() - indexWindowSize;
    if(first_window_index > beam.size() || first_window_index < minimumIndex)
            first_window_index = minimumIndex;
    unsigned second_window_index = first_window_index - indexWindowSize;
    if(second_window_index > beam.size() || second_window_index < minimumIndex)
            second_window_index = minimumIndex;
    if(first_window_index - second_window_index != indexWindowSize)
        return -1;
    
    for(unsigned int i = first_window_index; i < beam.size() ; i++)
        first_window_value += beam[i];
    for(unsigned int i = second_window_index; i < first_window_index ; i++)
        second_window_value += beam[i];

    
    //slide windows
    int j = second_window_index - 1;
    for(int i = first_window_index - 1; j >= (int)minimumIndex; i--, j--)
    {
        float diff_value = second_window_value - first_window_value * 3.0f;
        if(diff_value > best_window_difference)
        {
            best_window_difference = diff_value;
            best_window_pos = i;
            best_value_difference = second_window_value / indexWindowSize - first_window_value / indexWindowSize;
        }
        first_window_value+= beam[i];
        first_window_value-= beam[i+indexWindowSize];
        second_window_value+= beam[(unsigned int)j];
        second_window_value-= beam[(unsigned int)j+indexWindowSize];
    }
    
    if (best_window_pos > 0 && best_value_difference > minimumValue)
        return best_window_pos;
    else
        return -1;
}

std::vector<FeatureCandidate> FeatureExtraction::computeDerivativeFeatureCandidates(const std::vector< float >& beam)
{
    std::vector<FeatureCandidate> candidates;
    // campute and add derivative
    try
    {
        addToDerivativeHistory(beam, derivative_history_length);
    }
    catch (std::runtime_error e)
    {
        std::cerr << "Can't compute the derivative of this beam. " << e.what() << std::endl;
        return candidates;
    }

    // get the minimum signal of |history_length| derivatives
    min_derivative.clear();
    if(derivativeHistory.size() > 0)
    {
        for(std::vector<float>::const_iterator it = derivativeHistory.front()->begin(); it != derivativeHistory.front()->end(); it++)
        {
            min_derivative.push_back(*it);
        }
        std::list< std::vector<float>* >::iterator it = derivativeHistory.begin();
        it++;
        for(; it != derivativeHistory.end(); it++)
        {
            if((*it)->size() > min_derivative.size())
            {
                min_derivative.resize((*it)->size());
            }
            dsp::minimizeSignals<std::vector<float>::const_iterator, std::vector<float>::iterator>(min_derivative.begin(), min_derivative.end(), (*it)->begin(), (*it)->end(), min_derivative.begin());
        }
    }
    
    if(min_derivative.size() == 0)
        return candidates;
    
    // find the most likely obstacle position
    std::vector<float>::const_iterator it = min_derivative.end();
    do
    {
        it--;
        if(*it < 0.0f)
        {
            // analyze plain
            float plain_window_value = 0.0f;
            if(force_plain)
            {
                unsigned int index = it - min_derivative.begin();
                while(min_derivative[index] < 0.0f && index < min_derivative.size() - 2)
                    index++;
                unsigned int plain_len = min_derivative.size() * plain_length;
                for(unsigned int i = index; i < index + plain_len && i < min_derivative.size(); i++)
                {
                    plain_window_value += beam[i];
                }
                plain_window_value = plain_window_value / (float)plain_len;
            }
            
            // analyze positive and negative slope
            int count = 0;
            float mean_value = 0.0f;
            while(it > min_derivative.begin() && *it < 0.0f)
            {
                count++;
                mean_value -= *it;
                it--;
            }
            int position = it - min_derivative.begin();
            while(it >= min_derivative.begin() && *it == 0.0f)
            {
                it--;
            }
            while(it >= min_derivative.begin() && *it > 0.0f)
            {
                count++;
                mean_value += *it;
                it--;
            }
            if(count > 0)
            {
                mean_value = mean_value / (float)count;
                FeatureCandidate candidate;
                candidate.beam_index = position;
                candidate.mean_value = mean_value;
                candidate.plain_value = plain_window_value;
                candidates.push_back(candidate);
            }
        }
    }
    while(it > min_derivative.begin());
    
    for(unsigned int i = 0; i < candidates.size(); i++)
    {
        // correct signal moderation
        candidates[i].mean_value = candidates[i].mean_value * ((1.0f - signal_balancing) + ((signal_balancing * (float)candidates[i].beam_index) / (float)min_derivative.size()));
        // reduce signal weight in device noise area
        candidates[i].mean_value = candidates[i].mean_value * (1 + ( pow((SonarEnvironmentModel::device_noise_distribution((float)candidates[i].beam_index) / 255.0f), 0.5) * -1.0f ));
    }
    
    // compute plain threshold
    if(force_plain)
    {
        plain_window_threshold = 0.0f;
        int index_counter = 0;
        for(unsigned int i = 0; i < beam.size(); i++)
        {
            if(beam[i] > 1.0f)
            {
                index_counter++;
                plain_window_threshold += beam[i];
            }
        }
        plain_window_threshold = (plain_window_threshold / (float)index_counter) * plain_threshold;
        
        // remove candidates if plain value is to high
        for(std::vector<FeatureCandidate>::iterator it = candidates.begin(); it != candidates.end();)
        {
            if(it->plain_value > plain_window_threshold)
                it = candidates.erase(it);
            else
                it++;
        }
    }
    
    // find the best value
    int best_pos = -1;
    float best_value = value_threshold;
    float best_peak = 0.0f;
    for(unsigned int i = 0; i < candidates.size(); i++)
    {
        if(candidates[i].mean_value > best_value)
        {
            best_value = candidates[i].mean_value;
            best_pos = candidates[i].beam_index;
            best_peak = min_derivative[candidates[i].beam_index];
        }
    }
    
    // compute possibilities for the canditates
    for(unsigned int i = 0; i < candidates.size(); i++)
    {
        if(candidates[i].mean_value > value_threshold)
            candidates[i].probability = candidates[i].mean_value / best_value;
        else
            candidates[i].probability = 0.0;
    }
    
    // sort candidates
    std::sort(candidates.begin(), candidates.end());
    std::reverse(candidates.begin(), candidates.end());
    
    // update value threshold
    cooldown_threshold++;
    if(best_pos >= 0 || cooldown_threshold > 3)
    {
        cooldown_threshold = 0;
        best_values.push_back(best_value);
        sum_best_values += best_value;
        if(best_values.size() >= best_values_size)
        {
            sum_best_values -= *best_values.begin();
            best_values.erase(best_values.begin());
        }
        if(best_values.size() > best_values_size / 5)
        value_threshold = (sum_best_values / (float)best_values.size()) * feature_threshold;
    }
    
    return candidates;
}

void FeatureExtraction::setDerivativeFeatureConfiguration(const unsigned int& derivative_history_length, const float& feature_threshold, 
                                                              const unsigned int& best_values_size, const float& signal_balancing, 
                                                              const float& plain_length, const float& plain_threshold)
{
    this->derivative_history_length = derivative_history_length;
    this->feature_threshold = feature_threshold;
    this->best_values_size = best_values_size;
    this->signal_balancing = signal_balancing;
    if(plain_length > 0.0f)
    {
        this->force_plain = true;
        this->plain_length = plain_length;
        this->plain_threshold = plain_threshold;
    }
    else
    {
        this->force_plain = false;
    }
}

void FeatureExtraction::getDerivativeFeatureDebugData(std::vector< float >& minimum_derivative, float& value_threshold, float& plain_window_threshold)
{
    minimum_derivative = this->min_derivative;
    value_threshold = this->value_threshold;
    plain_window_threshold = this->plain_window_threshold;
}

void FeatureExtraction::enforceLines(std::vector<FeatureCandidate> &feature_candidates, const base::Angle& bearing, double spatial_resolution, unsigned beam_size)
{
    hough_group_id = hough_group_id % max_hough_history;
    
    // delete old entries of the same id
    for(std::list<HoughEntry>::iterator hough_entry_it = hough_entries.begin(); hough_entry_it != hough_entries.end() && hough_entry_it->beam_id == hough_group_id; hough_entry_it = hough_entries.erase(hough_entry_it));
    
    // calculate feature coordnates and intersections in hough space
    unsigned int candidate_count = 0;
    for(unsigned int i = 0; i < feature_candidates.size() && candidate_count < max_candidates_per_beam; i++)
    {
        if(feature_candidates[i].probability > 0.0)
        {
            candidate_count++;
            HoughEntry hough_entry;
            hough_entry.beam_id = hough_group_id;
            hough_entry.feature2d.x() = (double)feature_candidates[i].beam_index * spatial_resolution * cos(bearing.rad);
            hough_entry.feature2d.y() = (double)feature_candidates[i].beam_index * spatial_resolution * sin(bearing.rad);
            hough_entry.probability = feature_candidates[i].probability;
            hough_entries.push_back(hough_entry);
            
            // calculate intersections with the existent entries in hough space
            for(std::list<HoughEntry>::iterator hough_entry_it = hough_entries.begin(); hough_entry_it != hough_entries.end(); hough_entry_it++)
            {
                if(hough_entry.feature2d.x() != hough_entry_it->feature2d.x() && hough_entry.beam_id != hough_entry_it->beam_id)
                {
                    // note: intersection is in hough space (Theta,r)
                    base::Vector2d intersection = computeHoughSpaceIntersection(hough_entry.feature2d, hough_entry_it->feature2d);
                    assert(intersection.x() > -M_PI && intersection.x() <= M_PI);
                    assert(intersection.y() >= 0.0);
                    hough_entry_it->intersection_points.push_back(base::Vector3d(intersection.x(), intersection.y(), (hough_entry.probability + hough_entry_it->probability) * 0.5));
                }
            }
        }
    }
    hough_group_id++;
    
    // calculate the probability of each intersection using a guassian distribution
    const static base::Angle PI_3 = base::Angle::fromRad(M_PI / 3.0);
    std::list<base::Vector3d> best_lines;
    for(std::list<HoughEntry>::iterator hough_entry_it = hough_entries.begin(); hough_entry_it != hough_entries.end(); hough_entry_it++)
    {
        for(std::vector<base::Vector3d>::iterator intersection_it = hough_entry_it->intersection_points.begin(); intersection_it != hough_entry_it->intersection_points.end(); intersection_it++)
        {
            // skip intersections outside of +/-60° around the actual bearing
            base::Angle theta = base::Angle::fromRad(intersection_it->x());
            if(theta.isInRange(bearing + PI_3, bearing - PI_3))
                continue;
            
            double probability_of_intersection = 0.0;
            for(std::list<HoughEntry>::const_iterator hough_entry_it_2 = hough_entries.begin(); hough_entry_it_2 != hough_entries.end(); hough_entry_it_2++)
            {
                // skip intersections having their origin in the same beam
                if(hough_entry_it_2->beam_id == hough_entry_it->beam_id)
                    continue;
                
                for(std::vector<base::Vector3d>::const_iterator intersection_it_2 = hough_entry_it_2->intersection_points.begin(); intersection_it_2 != hough_entry_it_2->intersection_points.end(); intersection_it_2++)
                {
                    // skip intersections if they are to near (maybe use a e-function here)
                    if(intersection_it_2->y() < 1.5)
                        continue;
                    // increase probability of this intersection
                    else if(std::abs(intersection_it->x() - intersection_it_2->x()) > M_PI)
                    {
                        double shifted_theta = 0.0;
                        if(intersection_it_2->x() < 0.0)
                            shifted_theta = intersection_it_2->x() + M_PI * 2.0;
                        else
                            shifted_theta = intersection_it_2->x() - M_PI * 2.0;
                        probability_of_intersection += machine_learning::calc_gaussian<2>(VECTOR_XD(2)(intersection_it->x(), intersection_it->y()), hough_covariance, VECTOR_XD(2)(shifted_theta, intersection_it_2->y()));
                    }
                    else
                    {
                        probability_of_intersection += machine_learning::calc_gaussian<2>(VECTOR_XD(2)(intersection_it->x(), intersection_it->y()), hough_covariance, VECTOR_XD(2)(intersection_it_2->x(), intersection_it_2->y()));
                    }
                }
            }
            //TODO maybe us the candidate probability (intersection_it->z()) here in some rate to weight the probability_of_intersection
            intersection_it->z() = probability_of_intersection;
            // add to sorted list
            if(intersection_it->z() > 0.01)
            {
                std::list<base::Vector3d>::iterator lines_it = best_lines.begin();
                for(;lines_it != best_lines.end() && lines_it->z() <= intersection_it->z(); lines_it++);
                best_lines.insert(lines_it, *intersection_it);
            }
        }
    }
    
    // create debug data
    force_wall_pos.clear();
    for(std::list<base::Vector3d>::const_iterator it = best_lines.begin(); it != best_lines.end(); it++)
    {
        double theta = it->x();
        double r = it->y();
        base::Vector2d p1;
        p1.x() = r * cos(theta);
        p1.y() = r * sin(theta);
        double theta_2 = bearing.rad;
        double r_2 = r / cos(theta_2-theta);
        base::Vector2d p2;
        p2.x() = r_2 * cos(theta_2);
        p2.y() = r_2 * sin(theta_2);
        
        p2 = p2 - p1;
        force_wall_pos.push_back(base::Vector3d(p1.x(), p1.y(), it->z()));
        force_wall_pos.push_back(base::Vector3d(p2.x(), p2.y(), it->z()));
    }
    
    if(!best_lines.empty())
    {   
        double max_probability = best_lines.back().z() > 1.0 ? best_lines.back().z() : 1.0;
        // get index offsets
        std::vector<unsigned> beam_index_offsets;
        std::vector<double> index_probabilities;
        for(std::list<base::Vector3d>::reverse_iterator it = best_lines.rbegin(); it != best_lines.rend(); it++)
        {
            double theta = it->x();
            double r = it->y();
            double dist_on_beam = r / cos(bearing.rad - theta);
            int index_offset = (int)((dist_on_beam / spatial_resolution) + 0.5);
            if(index_offset >= beam_size || index_offset < 0.0)
                continue;
            beam_index_offsets.push_back((unsigned)index_offset);
            assert((it->z() / max_probability) <= 1.0);
            index_probabilities.push_back(it->z() / max_probability);
        }
        
        // create a probability mask of all estimated lines
        std::vector<double> line_probability_mask(beam_size);
        for(unsigned j = 0; j < beam_index_offsets.size(); j++)
        {
            VECTOR_XD(1) mean;
            mean << beam_index_offsets[j];
            for(unsigned i = beam_index_offsets[j]; i < line_probability_mask.size(); i++)
            {
                VECTOR_XD(1) index;
                index << i;
                double weight = machine_learning::calc_gaussian<1>(mean, beam_covariance / spatial_resolution, index);
                if(weight < 0.01)
                    break;
                line_probability_mask[i] += weight * index_probabilities[j]; 
            }
            
            for(int i = ((int)beam_index_offsets[j]) - 1; i >= 0; i--)
            {
                VECTOR_XD(1) index;
                index << i;
                double weight = machine_learning::calc_gaussian<1>(mean, beam_covariance / spatial_resolution, index);
                if(weight < 0.01)
                    break;
                line_probability_mask[i] += weight * index_probabilities[j]; 
            }
        }
        
        // find highest probability
        double highest_value = 0.0;
        unsigned best_enforce_line_index = 0;
        for(unsigned i = 0; i < line_probability_mask.size(); i++)
        {
            if(line_probability_mask[i] > highest_value)
            {
                highest_value = line_probability_mask[i];
                best_enforce_line_index = i;
            }
        }
        
        // normalize mask
        for(unsigned i = 0; i < line_probability_mask.size(); i++)
        {
            line_probability_mask[i] /= highest_value;
        }
        
        // apply new weighting to feature candidates
        for(unsigned i = 0; i < feature_candidates.size(); i++)
        {
            feature_candidates[i].probability = feature_candidates[i].probability * (1.0 - enforce_line_pos_rate) + line_probability_mask[feature_candidates[i].beam_index] * enforce_line_pos_rate;
        }
        
        // add new candidate with best enforce line weight
        if(best_enforce_line_index > 0)
        {
            FeatureCandidate best_enforce_line_candidate;
            best_enforce_line_candidate.beam_index = best_enforce_line_index;
            best_enforce_line_candidate.probability = line_probability_mask[best_enforce_line_index] * enforce_line_pos_rate;
            feature_candidates.push_back(best_enforce_line_candidate);
        }
        
        // sort features
        std::sort(feature_candidates.begin(), feature_candidates.end());
        std::reverse(feature_candidates.begin(), feature_candidates.end());
    }
}

void FeatureExtraction::setEnforceLinesConfiguration(unsigned int max_hough_history, unsigned int max_candidates_per_beam, double enforce_line_pos_rate)
{
    this->max_hough_history = max_hough_history;
    this->max_candidates_per_beam = max_candidates_per_beam;
    this->enforce_line_pos_rate = enforce_line_pos_rate;
}

void FeatureExtraction::getEnforceLinesDebugData(std::list< HoughEntry >& hough_entries, std::vector< base::Vector3d >& force_wall_pos)
{
    hough_entries = this->hough_entries;
    force_wall_pos = this->force_wall_pos;
}

void FeatureExtraction::filterCandidates(std::vector< FeatureCandidate >& feature_candidates, double probability_threshold, unsigned int average_length)
{
    // find the best value
    float best_probability = probability_history_treshold;
    for(unsigned int i = 0; i < feature_candidates.size(); i++)
    {
        if(feature_candidates[i].probability > best_probability)
        {
            best_probability = feature_candidates[i].probability;
            
        }
    }
    
    // update probability threshold
    probability_treshold_cooldown++;
    if(best_probability > probability_history_treshold || probability_treshold_cooldown > 5)
    {
        probability_treshold_cooldown = 0;
        probability_treshold_history.push_back(best_probability);
        probability_treshold_sum += best_probability;
        if(probability_treshold_history.size() >= average_length)
        {
            probability_treshold_sum -= *probability_treshold_history.begin();
            probability_treshold_history.erase(probability_treshold_history.begin());
        }
        if(probability_treshold_history.size() > average_length / 5)
            probability_history_treshold = (probability_treshold_sum / (float)probability_treshold_history.size()) * probability_threshold;
    }
    
    // filter candidates
    if(probability_history_treshold > 0.0)
    {
        for(std::vector<FeatureCandidate>::iterator it = feature_candidates.begin(); it != feature_candidates.end();)
        {
            if(it->probability < probability_history_treshold)
            {
                it = feature_candidates.erase(it);
            }
            else
            {
                it++;
            }
        }
    }
}

base::Vector2d FeatureExtraction::computeHoughSpaceIntersection(base::Vector2d& feature1, base::Vector2d& feature2)
{
    double theta = atan((feature2.x() - feature1.x())/(feature1.y() - feature2.y()));
    double r = feature1.x() * cos(theta) + feature1.y() * sin(theta);
    if(r < 0.0)
    {
        if(theta <= 0.0)
            theta = theta + M_PI;
        else
            theta = theta - M_PI;
        r = feature1.x() * cos(theta) + feature1.y() * sin(theta);
    }
    return base::Vector2d(theta, r);
}

void FeatureExtraction::addToDerivativeHistory(const std::vector< float >& beam, const unsigned int &history_length)
{
    std::vector<float> *derivative = new std::vector<float>(beam.size());
    dsp::derivativeSignal<std::vector<float>::const_iterator, std::vector<float>::iterator>(beam.begin(), beam.end(), derivative->begin(), 10, 0.1);
    derivativeHistory.push_front(derivative);
    while(derivativeHistory.size() > history_length)
    {
        std::vector<float> *old_beam = derivativeHistory.back();
        derivativeHistory.pop_back();
        delete old_beam;
    }
}

void FeatureExtraction::setBoundingBox(const double radius, const double sampling_interval, const int speed_of_sound)
{
    if (sampling_interval != 0.0 && speed_of_sound != 0)
        minimumIndex = (radius * 2) / (sampling_interval * speed_of_sound);
}

void FeatureExtraction::setMinResponseValue(const double minValue)
{
    this->minimumValue = minValue;
}

obstaclePoint FeatureExtraction::computeObstaclePoint(const int& index, const base::samples::SonarBeam& sonar_beam, const base::Orientation& orientation)
{
    double scanAngle = sonar_beam.bearing.rad;
    double distance = (double)index * sonar_beam.getSpatialResolution();
        
    Eigen::Vector3d wallPoint(distance,0,0);
    Eigen::Vector3d topPoint(0,0,1);
    
    wallPoint = orientation * wallPoint;
    topPoint = orientation * topPoint;
    
    Eigen::AngleAxisd rotate(scanAngle,topPoint);
    
    wallPoint = rotate * wallPoint;
        
    sonar_detectors::obstaclePoint obstaclePoint;
    obstaclePoint.position = wallPoint;
    obstaclePoint.time = sonar_beam.time;
    obstaclePoint.value = sonar_beam.beam[index];
    obstaclePoint.distance = distance;
    obstaclePoint.angle = base::Angle::fromRad(scanAngle + base::getYaw(orientation));
    
    return obstaclePoint;
}

base::samples::LaserScan FeatureExtraction::computeLaserScan(const int& index, const base::samples::SonarBeam& sonar_beam)
{
    base::samples::LaserScan laser_scan;
    laser_scan.reset();
    laser_scan.minRange = 1000;
    laser_scan.maxRange = (uint32_t)(1000 * (sonar_beam.beam.size()-1) * sonar_beam.getSpatialResolution());
    laser_scan.angular_resolution = 0.0;
    laser_scan.speed = 0.0;
    laser_scan.start_angle = sonar_beam.bearing.rad;
    laser_scan.time = sonar_beam.time;
    
    if(index >= 0)
    {
        laser_scan.ranges.push_back((uint32_t)(index * sonar_beam.getSpatialResolution() * 1000));
        laser_scan.remission.push_back(sonar_beam.beam[index]);
    }
    else
    {
        laser_scan.ranges.push_back(base::samples::TOO_FAR);
    }
    
    return laser_scan;
}

}
