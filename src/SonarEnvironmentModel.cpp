#include "SonarEnvironmentModel.hpp"

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
    
}
