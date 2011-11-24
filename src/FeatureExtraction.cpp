#include "FeatureExtraction.hpp"

namespace sonar_detectors
{
    
SonarEnvironmentModel::SonarEnvironmentModel() : 
                        orientation(base::Quaterniond::Identity()),
                        sampling_interval(0),
                        beamwidth_vertical(0),
                        speed_of_sound(0),
                        distance_to_ground(0),
                        distance_to_surface(0),
                        distance_to_avalon_model(0.15)
{
    avalon_model.push_back(base::Vector2d(0, 0.4));
    avalon_model.push_back(base::Vector2d(M_PI_4, 0.35));
    avalon_model.push_back(base::Vector2d(M_PI_2, 0.15));
    avalon_model.push_back(base::Vector2d(7 * M_PI_8, 1.05));
    avalon_model.push_back(base::Vector2d(M_PI, 1.10));
}

void SonarEnvironmentModel::getExpectedObstaclePositions(const double beam_angle, int& pos_auv, int& pos_ground, int& pos_surface)
{
    // transform beam angle value to range 1..-1
    double linearized_beam_angle = beam_angle;
    if(beam_angle < 0)
        linearized_beam_angle = linearized_beam_angle * -1;
    linearized_beam_angle = ((linearized_beam_angle / M_PI_2) - 1) * -1;
    
    double auv_pitch_angle = orientation.toRotationMatrix().eulerAngles(2,1,0)[1];
    double beamwidth_vertical_up = (beamwidth_vertical * 0.5) - auv_pitch_angle * linearized_beam_angle;
    double beamwidth_vertical_down = (beamwidth_vertical * 0.5) + auv_pitch_angle * linearized_beam_angle;
    
    pos_auv = getAUVModelConstrains(beam_angle, beamwidth_vertical_down);
    pos_ground = getExpectedObstaclePosition(beamwidth_vertical_down, distance_to_ground);
    pos_surface = getExpectedObstaclePosition(beamwidth_vertical_up, distance_to_surface);
}

int SonarEnvironmentModel::getExpectedObstaclePosition(const double beamwidth_vertical_angle, const double distance_to_plain)
{
    if(distance_to_plain <= 0 || beamwidth_vertical_angle <= 0)
        return -1;
    
    double distance = distance_to_plain / tan(beamwidth_vertical_angle);
    
    int pos = (int)(distance / (sampling_interval * 0.5 * speed_of_sound));
    return pos;
}

int SonarEnvironmentModel::getAUVModelConstrains(const double beam_angle, double& beamwidth_vertical_down)
{
    double positive_beam_angle = beam_angle;
    if(positive_beam_angle < 0)
        positive_beam_angle = positive_beam_angle * -1;
    
    double distance = 0;
    if(angle_to_edge_distance.find(positive_beam_angle) == angle_to_edge_distance.end())
    {
        int i = 0;
        while(i < avalon_model.size() - 1 && positive_beam_angle > avalon_model[i+1].x())
        {
            i++;
        }
        if(i >= avalon_model.size() - 1)
            return -1;
        
        double angle_diff = avalon_model[i+1].x() - avalon_model[i].x();
        double reduced_beam_angle = positive_beam_angle - avalon_model[i].x();
        double dist_diff = avalon_model[i+1].y() - avalon_model[i].y();
        distance = (dist_diff / angle_diff) * reduced_beam_angle;
        distance = distance + avalon_model[i].y();
        angle_to_edge_distance.insert(std::make_pair<double, double>(positive_beam_angle, distance));
    }
    else
    {
        distance = angle_to_edge_distance.at(positive_beam_angle);
    }
    
    double max_angle = atan2(distance_to_avalon_model, distance);
    if (beamwidth_vertical_down > max_angle)
    {
        beamwidth_vertical_down = max_angle;
        int pos = (int)(distance / (sampling_interval * 0.5 * speed_of_sound));
        return pos;
    }
    return -1;
}
    
void SonarEnvironmentModel::updateSonarBeamProperties(const double sampling_interval, const float beamwidth_vertical, const float speed_of_sound)
{
    this->sampling_interval = sampling_interval;
    this->beamwidth_vertical = beamwidth_vertical;
    this->speed_of_sound = speed_of_sound;
}

void SonarEnvironmentModel::updateAUVOrientation(const base::Quaterniond orientation)
{
    this->orientation = orientation;
}

void SonarEnvironmentModel::updateDistanceToGround(const double distance)
{
    if(distance < 0)
        distance_to_ground = 0;
    else 
        distance_to_ground = distance;
}

void SonarEnvironmentModel::updateDistanceToSurface(const double distance)
{
    if(distance < 0)
        distance_to_surface = 0;
    else 
        distance_to_surface = distance;
}




    
FeatureExtraction::FeatureExtraction() :
                    minimumIndex(0),
                    minimumValue(20),
                    indexWindowSize(50)
{
    
}
    
int FeatureExtraction::getFeatureGlobalMaxima(const std::vector<float>& beam)
{    
    // cut up noise
    while(beam[minimumIndex] > 5 && minimumIndex < beam.size())
        minimumIndex++;

    float act_window_value = 0;
    float best_window_value = 0;
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

int FeatureExtraction::getFeatureHighestWaveFromBehind(const std::vector< float >& beam)
{
    if(beam.size() == 0)
        return -1;
        
    float act_window_value = 0;
    float best_window_value = 0;
    unsigned int best_window_pos = 0;
    
    //fill window
    unsigned begin_index = beam.size() - indexWindowSize;
    if(begin_index > beam.size() || begin_index < minimumIndex)
            begin_index = minimumIndex;
    for(unsigned int i = begin_index; i < beam.size() ; i++)
        act_window_value += beam[i];
    
    //slide window 
    for(int i = begin_index - 1; i >= minimumIndex; i--)
    {
        if(act_window_value > best_window_value)
        {
            best_window_value = act_window_value;
            best_window_pos = i;
        }
        else if(best_window_value > minimumValue * 10 && act_window_value < best_window_value * 0.5)
        {
            break;
        }
        act_window_value+= beam[i];
        act_window_value-= beam[i+indexWindowSize];
    }
    
    //find maximum insight the best window
    float best_val = 0;
    int best_index = -1;
    for(unsigned int i = best_window_pos; i < beam.size() || i < best_window_pos + indexWindowSize; i++)
    {
        if(beam[i]> best_val)
        {
            best_val = beam[i];
            best_index = (int)i;
        }
    }
    
    if (best_index > 0 && beam[best_index] > minimumValue)
        return best_index;
    else
        return -1;
}

int FeatureExtraction::getFeatureMaximalLevelDifference(const std::vector< float >& beam)
{
    if(beam.size() == 0)
        return -1;
    
    float first_window_value = 0;
    float second_window_value = 0;
    float best_window_difference = -10000;
    float best_value_difference = 0;
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
        float diff_value = second_window_value - first_window_value * 3;
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

std::vector<float> FeatureExtraction::smoothFilter(const std::vector<uint8_t>& beam)
{
    if(beam.size() == 0)
        return std::vector<float>();
        
    std::vector<float> filtered_beam;
    filtered_beam.push_back(beam[0] * 0.5);
    for(unsigned int i = 1; i < beam.size(); i++)
    {
        filtered_beam.push_back((beam[i] + beam[i-1]) * 0.5);
    }
    return filtered_beam;
}

std::vector<float> FeatureExtraction::balancePointFilter(const std::vector<uint8_t>& beam)
{
    std::vector<float> filtered_beam;
    int window_value = 0;
    
    //fill window
    unsigned window_index = 0;
    for(unsigned int i = window_index; i < indexWindowSize && i < beam.size(); i++)
        window_value += beam[i];
    
    for(int i = 0; i < indexWindowSize / 2; i++)
        filtered_beam.push_back(0);

    //slide window
    for(int i = window_index; i < ((int)beam.size()) - indexWindowSize; i++)
    {
        filtered_beam.push_back((uint8_t)(window_value / indexWindowSize));
        window_value+= beam[i+indexWindowSize];
        window_value-= beam[i];
    }

    for(int i = 0; i < indexWindowSize / 2; i++)
        filtered_beam.push_back(0);

    return filtered_beam;
}

void FeatureExtraction::removeInfluence(std::vector< float >& beam)
{
    unsigned int start_pos = beam.size();
    for(int i = beam.size() - 1; i > minimumIndex; i--)
    {
        if(beam[i] < 3)
        {
            for(int j = i; j < start_pos; j++)
            {
                beam[j] = 0;
            }
            start_pos = i;
        }
    }
}

void FeatureExtraction::setBoundingBox(const double radius, const double sampling_interval, const int speed_of_sound)
{
    if (sampling_interval != 0.0 && speed_of_sound != 0)
        minimumIndex = (radius * 2) / (sampling_interval * speed_of_sound);
    if (minimumIndex < indexWindowSize * 0.5)
        minimumIndex = indexWindowSize * 0.5;
}

void FeatureExtraction::setMinResponseValue(double minValue)
{
    this->minimumValue = minValue;
}
    
}
