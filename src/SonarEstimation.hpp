#ifndef SONAR_ESTIMATION_HPP_
#define SONAR_ESTIMATION_HPP_

#include "SonarDetectorTypes.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/LaserScan.hpp>
#include <base/Float.hpp>
#include <vector>

namespace sonar_detectors
{
    class SonarEstimation
    {
    public:
	void updateFeature(const base::samples::LaserScan &feature, const base::Angle &global_heading)
	{
	    updateFeature(feature, Eigen::Affine3d(Eigen::AngleAxisd(global_heading.getRad(), Eigen::Vector3d::UnitZ())));
	}
	
        void updateFeature(const base::samples::LaserScan &feature, const Eigen::Affine3d &featureInOdometry = Eigen::Affine3d::Identity())
        {
            // check if vector is empty
            if(feature.ranges.empty())
                return;
            
            // check if the LaserScan contains only ranges in the same angle
            if(feature.angular_resolution != 0.0)
            {
                throw std::runtime_error("The LaserScan must contain only features in the same angle. The angular resolution should be 0.");
            }
            
            // find scan direction
            switched_scan_direction = false;
            if(!base::isNaN<double>(last_local_angle.rad))
	    {
		double angle_diff = (base::Angle::fromRad(feature.start_angle) - last_local_angle).getRad();
		
		ScanDirection new_direction = Unknown;
		if(angle_diff > 0.0)
		    new_direction = Left;
		else if(angle_diff < 0.0)
		    new_direction = Right;
		
		if(new_direction != current_scan_direction)
		{
		    if(current_scan_direction != Unknown && new_direction != Unknown)
			switched_scan_direction = true;
		    current_scan_direction = new_direction;
		}
	    }
	    last_local_angle = base::Angle::fromRad(feature.start_angle);
	    
	    // set zone state
            switched_zone = false;
	    if((start_angle - base::Angle::fromRad(feature.start_angle)).getRad() > 0.0 
		&& (end_angle - base::Angle::fromRad(feature.start_angle)).getRad() < 0.0)
	    {
		if(!inside_estimation_zone)
		{
		    inside_estimation_zone = true;
		    switched_zone = true;
		}
	    }
	    else
	    {
		if(inside_estimation_zone)
		{
		    inside_estimation_zone = false;
		    switched_zone = true;
		}
	    }
	    
            // set global angles
            this->global_heading = base::Angle::fromRad(base::getYaw(base::Orientation(featureInOdometry.linear())));
            this->global_start_angle = start_angle + global_heading;
            this->global_end_angle = end_angle + global_heading;
	    
	    updateFeatureIntern(feature, featureInOdometry);
	    /*
            if(global_heading.rad != 0.0)
            {
                base::samples::LaserScan global_feature;
                global_feature.angular_resolution = feature.angular_resolution;
                global_feature.maxRange = feature.maxRange;
                global_feature.minRange = feature.minRange;
                global_feature.ranges = feature.ranges;
                global_feature.remission = feature.remission;
                global_feature.speed = feature.speed;
                global_feature.time = feature.time;
                global_feature.start_angle = base::Angle::normalizeRad(feature.start_angle + global_heading.rad);
                updateFeatureIntern(global_feature);
            }
            else
            {
                updateFeatureIntern(feature);
            }
            */
        };
        
        void setEstimationZone(const base::Angle &start_angle, const base::Angle &end_angle)
        {
            this->start_angle = start_angle;
            this->end_angle = end_angle;
        };
        
    protected:
	enum ScanDirection
	{
	    Left,
	    Right,
	    Unknown
	};
	
        SonarEstimation()
        {
            global_heading = base::Angle::fromRad(0.0);
            global_start_angle = base::Angle::fromRad(0.0);
            global_end_angle = base::Angle::fromRad(0.0);
            start_angle = base::Angle::fromRad(0.0);
            end_angle = base::Angle::fromRad(0.0);
	    switched_scan_direction = false;
	    switched_zone = false;
	    inside_estimation_zone = false;
	    current_scan_direction = Unknown;
	    last_local_angle.rad = base::NaN<double>();
        };
        
        virtual void updateFeatureIntern(const base::samples::LaserScan &feature, const Eigen::Affine3d &featureInOdometry) = 0;
        
    protected:
        base::Angle global_heading;
        base::Angle global_start_angle;
        base::Angle global_end_angle;
	bool switched_scan_direction;
	bool switched_zone;
	bool inside_estimation_zone;
	ScanDirection current_scan_direction;
    private:
        base::Angle start_angle;
        base::Angle end_angle;
	base::Angle last_local_angle;
    };
}

#endif