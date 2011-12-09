#include "FeatureExtraction.hpp"
#include <dsp_acoustics/FIRFilter.h>
#include <iostream>

namespace sonar_detectors
{
    
FeatureExtraction::FeatureExtraction() :
                    minimumIndex(0),
                    minimumValue(20.0)
{
}

FeatureExtraction::~FeatureExtraction()
{
    for(std::list< std::vector<float>* >::iterator it = derivativeHistory.begin(); it != derivativeHistory.end(); it++)
    {
        delete *it;
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
    for(int i = first_window_index - 1; j >= minimumIndex; i--, j--)
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
        second_window_value+= beam[j];
        second_window_value-= beam[j+indexWindowSize];
    }
    
    if (best_window_pos > 0 && best_value_difference > minimumValue)
        return best_window_pos;
    else
        return -1;
}

int FeatureExtraction::getFeatureDerivativeHistory(const std::vector< float >& beam, const unsigned int &history_length, const float &threshold, const bool &indoor_mode)
{
    try
    {
        addToDerivativeHistory(beam, history_length);
    }
    catch (std::runtime_error e)
    {
        std::cerr << "Can't compute the derivative of this beam. " << e.what() << std::endl;
        return -1;
    }

    std::vector<float> min_derivative;
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
    
    int best_pos = -1;
    float best_peak = 0.0f;
    float best_pos_plain_window_value = 1000.0f;
    if(indoor_mode)
    {
        std::vector<float>::const_iterator it = min_derivative.begin();
        do
        {
            it++;
            if(*it > threshold)
            {
                int index = it - min_derivative.begin();
                // analyze plain
                while(min_derivative[index] > 0 && index > 0)
                    index--;
                float plain_window_value = 0.0f;
                int plain_len = min_derivative.size() * 0.05;
                if(index - plain_len < 0)
                    continue;
                for(int i = index; i < index - plain_len; i--)
                {
                    plain_window_value += beam[i];
                }
                plain_window_value = plain_window_value / plain_len;
                // get highest peak
                index = it - min_derivative.begin();
                int highest_peak = index;
                while(min_derivative[index] > 0 && index < min_derivative.size() - 2)
                {
                    if (min_derivative[index] > min_derivative[highest_peak])
                        highest_peak = index;
                    index++;
                }
                highest_peak = index;
                
                if((best_pos_plain_window_value > plain_window_value || min_derivative[highest_peak] > best_peak * 2.0) && plain_window_value < threshold)
                {
                    best_pos_plain_window_value = plain_window_value;
                    best_peak = min_derivative[highest_peak];
                    best_pos = highest_peak;
                }
            }
        }
        while(it < min_derivative.end() - 1);
    }
    else
    {
        std::vector<float>::const_iterator it = min_derivative.end();
        do
        {
            it--;
            if(*it < threshold * -1.0f)
            {
                int index = it - min_derivative.begin();
                // analyze plain
                while(min_derivative[index] < 0 && index < min_derivative.size() - 2)
                    index++;
                float plain_window_value = 0.0f;
                int plain_len = min_derivative.size() * 0.05;
                if(index + plain_len > min_derivative.size())
                    continue;
                for(int i = index; i < index + plain_len; i++)
                {
                    plain_window_value += beam[i];
                }
                plain_window_value = plain_window_value / plain_len;
                
                // get smallest peak
                index = it - min_derivative.begin();
                int smallest_peak = index;
                while(min_derivative[index] < 0 && index > 0)
                {
                    if (min_derivative[index] < min_derivative[smallest_peak])
                        smallest_peak = index;
                    index--;
                }
                
                if((best_pos_plain_window_value > plain_window_value || min_derivative[smallest_peak] < best_peak * 2.0) && plain_window_value < threshold * 2.0)
                {
                    best_pos_plain_window_value = plain_window_value;
                    best_peak = min_derivative[smallest_peak];
                    best_pos = smallest_peak;
                }
            }
        }
        while(it >= min_derivative.begin());
    }
    return best_pos;
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
