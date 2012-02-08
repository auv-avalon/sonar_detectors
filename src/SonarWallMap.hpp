#ifndef SONAR_WALL_MAP_HPP_
#define SONAR_WALL_MAP_HPP_

#include <vector>
#include <base/angle.h>
#include <stdint.h>
#include <stdlib.h>

namespace sonar_detectors
{

class SonarWallMap
{
public:
    SonarWallMap()
    {
        angular_resolution = 0.0;
        cool_down_factor = 0.01;
        sample_counter = 0;
        healthy_sample_count = 100;
    };

    void updateAngle(double angle)
    {
        if(angular_resolution > 0)
        {
            if(sample_counter < healthy_sample_count)
                sample_counter++;
            unsigned index = getIndexOfAngle(base::Angle::fromRad(angle).rad);
            
            wall_possibility[index] += 1.0f;
            
            for(unsigned int i = 0; i < getIndexOfAngle(-M_PI); i++)
            {
                wall_possibility[i] -= wall_possibility[i] * cool_down_factor;
            }
        }
    };
    
    void setResolution(unsigned int grid_resolution)
    {
        if(grid_resolution <= 0)
            throw std::runtime_error("The grid resolution must be greater than 0.");
        clear();
        wall_possibility.resize(grid_resolution, 0.0f);
        angular_resolution = (2.0 * M_PI) / (double)grid_resolution;
    };
    
    /**
     * Delete entries
     */
    void clear()
    {
        wall_possibility.clear();
        sample_counter = 0;
    };
    
    std::vector<float> getWallPossibilities()
    {
        return wall_possibility;
    };
    
    base::Angle getAngleForBestWall()
    {
        int best_pos = 0;
        for(unsigned i = 0; i < getIndexOfAngle(-M_PI); i++)
        {
            if(wall_possibility[i] >= wall_possibility[best_pos])
            {
                best_pos = i;
            }
        }
        return base::Angle::fromRad(getAngleOfIndex(best_pos));
    };
    
    bool isHealthy()
    {
        return sample_counter >= healthy_sample_count ? true : false;
    };
    
protected:
    double getAngleOfIndex(const unsigned int index)
    {
        return M_PI - (index * angular_resolution + angular_resolution * 0.5);
    };
    
    unsigned int getIndexOfAngle(const double &angle)
    {
        if(angular_resolution <= 0)
            throw std::runtime_error("Can't get the index if the angular resolution is 0 or less.");
        return (unsigned)((angle - M_PI) / -angular_resolution);
    }
    
    std::vector<float> wall_possibility;
    double angular_resolution;
    float cool_down_factor;
    unsigned int sample_counter;
    unsigned int healthy_sample_count;
};
    
};

#endif