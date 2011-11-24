#include "FeatureExtraction.hpp"

namespace sonar_detectors
{
    
FeatureExtraction::FeatureExtraction() :
                    minimumIndex(0),
                    minimumValue(20.0),
                    indexWindowSize(50)
{
    
}
    
int FeatureExtraction::getFeatureGlobalMaxima(const std::vector<float>& beam)
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

int FeatureExtraction::getFeatureMaximalLevelDifference(const std::vector< float >& beam)
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
    if (minimumIndex < indexWindowSize * 0.5)
        minimumIndex = indexWindowSize * 0.5;
}

void FeatureExtraction::setMinResponseValue(const double minValue)
{
    this->minimumValue = minValue;
}
    
}
