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
                    minimumValue(20.0)
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
}

FeatureExtraction::~FeatureExtraction()
{
    for(std::list< std::vector<float>* >::iterator it = derivativeHistory.begin(); it != derivativeHistory.end(); it++)
    {
        derivativeHistory.erase(it);
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

int FeatureExtraction::getFeatureDerivativeHistory(const std::vector< float >& beam)
{
    // campute and add derivative
    try
    {
        addToDerivativeHistory(beam, derivative_history_length);
    }
    catch (std::runtime_error e)
    {
        std::cerr << "Can't compute the derivative of this beam. " << e.what() << std::endl;
        return -1;
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
        return -1;
    
    // find the most likely obstacle position
    std::vector<float>::const_iterator it = min_derivative.end();
    possible_positions.clear();
    mean_values.clear();
    plain_values.clear();
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
                mean_values.push_back(mean_value);
                possible_positions.push_back(position);
                plain_values.push_back(plain_window_value);
            }
        }
    }
    while(it > min_derivative.begin());
    
    for(unsigned int i = 0; i < possible_positions.size(); i++)
    {
        // correct signal moderation
        mean_values[i] = mean_values[i] * ((1.0f - signal_balancing) + ((signal_balancing * (float)possible_positions[i]) / (float)min_derivative.size()));
        // reduce signal weight in device noise area
        mean_values[i] = mean_values[i] * (1 + ( pow((SonarEnvironmentModel::device_noise_distribution((float)possible_positions[i]) / 255.0f), 0.5) * -1.0f ));
    }
    
    // compute plain threshold
    plain_window_threshold = 100.0f;
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
    }
    
    // find the best value
    int best_pos = -1;
    float best_value = value_threshold;
    float best_peak = 0.0f;
    for(unsigned int i = 0; i < possible_positions.size(); i++)
    {
        if(mean_values[i] > best_value && plain_values[i] < plain_window_threshold)
        {
            best_value = mean_values[i];
            best_pos = possible_positions[i];
            best_peak = min_derivative[possible_positions[i]];
        }
    }
    
    // update value threshold
    cooldown_threshold++;
    if(best_pos >= 0 || (cooldown_threshold%2) == 2)
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
    
    return best_pos;
}

void FeatureExtraction::featureDerivativeHistoryConfiguration(const unsigned int& derivative_history_length, const float& feature_threshold, 
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

void FeatureExtraction::getFDHDebugData(std::vector< float >& minimum_derivative, float& value_threshold, float& plain_window_threshold, std::vector< int >& candidates, std::vector< float >& candidate_mean_value, std::vector< float >& candidate_plain_value)
{
    minimum_derivative = this->min_derivative;
    value_threshold = this->value_threshold;
    plain_window_threshold = this->plain_window_threshold;
    candidates = this->possible_positions;
    candidate_mean_value = this->mean_values;
    candidate_plain_value = this->plain_values;
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

std::vector< float > FeatureExtraction::convertBeam(const std::vector< uint8_t >& beam)
{
    std::vector<float> converted_beam;
    for(unsigned int i = 1; i < beam.size(); i++)
    {
        converted_beam.push_back((float)beam[i]);
    }
    return converted_beam;
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

}
