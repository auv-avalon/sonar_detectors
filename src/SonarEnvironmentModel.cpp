#include "SonarEnvironmentModel.hpp"
#include <dsp_acoustics/FIRFilter.h>

namespace sonar_detectors
{
    
//noise distributions variables
static float gaussian_ground_sigma;
static float gaussian_ground_u;
static float gaussian_ground_k;
static float gaussian_surface_sigma;
static float gaussian_surface_u;
static float gaussian_surface_k;
static float device_noise_sigma;
    
const static float sqrt_2_PI = sqrt(2.0f*M_PI);
    
SonarEnvironmentModel::SonarEnvironmentModel() : 
                        distance_to_ground(0),
                        distance_to_surface(0),
                        distance_to_avalon_model(0.15),
                        orientation(base::Quaterniond::Identity()),
                        sampling_interval(0),
                        beamwidth_vertical(0),
                        speed_of_sound(0)
{
    // abstract plain model of avalon
    avalon_model.push_back(base::Vector2d(0, 0.4));
    avalon_model.push_back(base::Vector2d(M_PI_4, 0.35));
    avalon_model.push_back(base::Vector2d(M_PI_2, 0.15));
    avalon_model.push_back(base::Vector2d(7 * M_PI_8, 1.05));
    avalon_model.push_back(base::Vector2d(M_PI, 1.10));
    
    // noise distributions variables
    device_noise_first_local_min_pos = 0;
    device_noise_sigma = 1.0f;
    gaussian_surface_sigma = 1.0f;
    gaussian_surface_u = 0.0f;
    gaussian_surface_k = 1.0f;
    gaussian_ground_sigma = 1.0f;
    gaussian_ground_u = 0.0f;
    gaussian_ground_k = 1.0f;
    // noise distributions config
    find_first_min_windowsize = 6;
    device_noise_minpos_update_rate = 0.05f;
    device_noise_grow_update_rate = 0.01f;
    gaussian_surface_update_rate = 0.01f;
    gaussian_ground_update_rate = 0.001f;
}

void SonarEnvironmentModel::updateNoiseDistributionValues(const double beam_angle, const std::vector< float > &beam)
{
    int pos_auv, pos_ground, pos_surface;
    getExpectedObstaclePositions(beam_angle, pos_auv, pos_ground, pos_surface);

    // correct device noise sigma
    if(pos_auv >= 0)
    {
        std::vector<float>::const_iterator device_noise_new_local_min = dsp::findFirstRightLocalMin<std::vector<float>::const_iterator>(beam.begin() + pos_auv, beam.end() - (beam.size() * 0.7), find_first_min_windowsize);
        if(device_noise_new_local_min != beam.begin())
        {
            device_noise_first_local_min_pos = device_noise_first_local_min_pos * (1-device_noise_minpos_update_rate) + (device_noise_new_local_min - beam.begin()) * device_noise_minpos_update_rate;
            if(device_noise_first_local_min_pos < pos_auv)
                device_noise_first_local_min_pos = pos_auv;
            
            float mean_diff = 0.0f;
            for(int i = 0; i < device_noise_first_local_min_pos; i++)
            {
                mean_diff += (beam[i] > device_noise_distribution(i)) ? beam[i] - device_noise_distribution(i) : 0.0f;
            }
            mean_diff = mean_diff / device_noise_first_local_min_pos;
            
            float diff = -1.0f;
            if(mean_diff > 0.0f)
                diff = mean_diff;
            
            device_noise_sigma += (device_noise_grow_update_rate * diff);
            if(device_noise_sigma < 1.0f) device_noise_sigma = 1.0f;
        }
    }
    
    // correct surface gaussian sigma
    if(pos_surface >= 0)
    {
        gaussian_surface_u = (float)pos_surface;
        std::vector<float>::const_iterator gaussian_surface_local_min = dsp::findFirstRightLocalMin<std::vector<float>::const_iterator>(beam.begin() + pos_surface, beam.end(), find_first_min_windowsize);
        float diff_surface = *gaussian_surface_local_min - gaussian_distribution_surface(gaussian_surface_local_min - beam.begin());
        gaussian_surface_sigma += (gaussian_surface_update_rate * diff_surface);
        if(gaussian_surface_sigma < 1.0f) gaussian_surface_sigma = 1.0f;
        // correct surface gaussian k
        diff_surface = beam[pos_surface] - gaussian_distribution_surface(pos_surface);
        gaussian_surface_k += (gaussian_surface_update_rate * diff_surface);
        if(gaussian_surface_k < 1.0f) gaussian_surface_k = 1.0f;
        //TODO improve the relation between height and width of the gauss distributions
        //else if (gaussian_surface_k > gaussian_surface_sigma) gaussian_surface_k = gaussian_surface_sigma;
    }
    
    // correct ground gaussian sigma
    if(pos_ground >= 0)
    {
        gaussian_ground_u = (float)pos_ground;
        std::vector<float>::const_iterator gaussian_ground_local_min = dsp::findFirstRightLocalMin<std::vector<float>::const_iterator>(beam.begin() + pos_ground, beam.end(), find_first_min_windowsize);
        float diff_ground = *gaussian_ground_local_min - gaussian_distribution_ground(gaussian_ground_local_min - beam.begin());
        gaussian_ground_sigma += (gaussian_ground_update_rate * diff_ground);
        if(gaussian_ground_sigma < 1.0f) gaussian_ground_sigma = 1.0f;
        // correct ground gaussian k
        diff_ground = beam[pos_ground] - gaussian_distribution_ground(pos_ground);
        gaussian_ground_k += (gaussian_ground_update_rate * diff_ground);
        if(gaussian_ground_k < 1.0f) gaussian_ground_k = 1.0f;
    }
}

void SonarEnvironmentModel::getExpectedObstaclePositions(const double beam_angle, int& pos_auv, int& pos_ground, int& pos_surface)
{
    // transform beam angle value to range 1..-1
    double linearized_beam_angle = beam_angle;
    if(beam_angle < 0.0)
        linearized_beam_angle = linearized_beam_angle * -1.0;
    linearized_beam_angle = ((linearized_beam_angle / M_PI_2) - 1.0) * -1.0;
    
    double auv_pitch_angle = orientation.toRotationMatrix().eulerAngles(2,1,0)[1];
    double beamwidth_vertical_up = (beamwidth_vertical * 0.5) - auv_pitch_angle * linearized_beam_angle;
    double beamwidth_vertical_down = (beamwidth_vertical * 0.5) + auv_pitch_angle * linearized_beam_angle;
    
    pos_auv = getAUVModelConstrains(beam_angle, beamwidth_vertical_down);
    pos_ground = getExpectedObstaclePosition(beamwidth_vertical_down, distance_to_ground);
    pos_surface = getExpectedObstaclePosition(beamwidth_vertical_up, distance_to_surface);
}

int SonarEnvironmentModel::getExpectedObstaclePosition(const double beamwidth_vertical_angle, const double distance_to_plain)
{
    if(distance_to_plain <= 0.0 || beamwidth_vertical_angle <= 0.0)
        return -1;
    
    double distance = distance_to_plain / tan(beamwidth_vertical_angle);
    
    int pos = (int)(distance / (sampling_interval * 0.5 * speed_of_sound));
    return pos;
}

int SonarEnvironmentModel::getAUVModelConstrains(const double beam_angle, double& beamwidth_vertical_down)
{
    double positive_beam_angle = beam_angle;
    if(positive_beam_angle < 0.0)
        positive_beam_angle = positive_beam_angle * -1.0;
    
    double distance = 0.0;
    if(angle_to_edge_distance.find(positive_beam_angle) == angle_to_edge_distance.end())
    {
        unsigned int i = 0;
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
    return 0;
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
    if(distance < 0.0)
        distance_to_ground = 0.0;
    else 
        distance_to_ground = distance;
}

void SonarEnvironmentModel::updateDistanceToSurface(const double distance)
{
    if(distance < 0)
        distance_to_surface = 0.0;
    else 
        distance_to_surface = distance;
}

float SonarEnvironmentModel::device_noise_distribution(float x)
{
    return 255.0f * exp(-x/device_noise_sigma);
}

float SonarEnvironmentModel::gaussian_distribution(const float &x, const float &sigma, const float &mu, const float &k)
{
    return (k*100.0f/(sigma * sqrt_2_PI)) * exp(-0.5f*(pow((x-mu) / sigma, 2)));
}

float SonarEnvironmentModel::gaussian_distribution_ground(float x)
{
    return gaussian_distribution(x, gaussian_ground_sigma, gaussian_ground_u, gaussian_ground_k);
}

float SonarEnvironmentModel::gaussian_distribution_surface(float x)
{
    return gaussian_distribution(x, gaussian_surface_sigma, gaussian_surface_u, gaussian_surface_k);
}
    
}