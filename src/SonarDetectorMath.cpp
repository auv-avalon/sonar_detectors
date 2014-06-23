#include "SonarDetectorMath.hpp"
#include <opencv/cv.h>
#include <base/angle.h>

namespace sonar_detectors
{

/**
 * Uses the ransac algorithm to estimate walls in a 3d point cloud.
 * Mostly one wall, but in corners it trys to estimate two walls.
 * 
 * @param pointCloud - the 3d point cloud
 * @param iterations - number of iterations
 * @param threshold - max inlier distance to the model
 * @param fit_rate -  min count inliers of the point cloud in percent, so it is a valid model
 * @param best_model - best model that could be found
 * @return best_error, outlier in percent. if returns 1.0, no valid model could be found
 */
double wallRansac(const std::vector< base::Vector3d >& pointCloud, unsigned iterations, double threshold, double fit_rate, std::vector< std::pair< base::Vector3d, base::Vector3d > >& best_models)
{
    best_models.clear();
    double best_error = 1.0;
    
    if (pointCloud.size() > 0)
    {
        // first wall
        std::vector< base::Vector3d > outlier_wall1;
        std::pair< base::Vector3d, base::Vector3d > model_wall1;
        double fitrate_wall1 = ransac(pointCloud, iterations, threshold, outlier_wall1, model_wall1);
        
        if (fitrate_wall1 > 0.0)
        {
            /*
            // second wall
            std::vector< base::Vector3d > outlier_wall2;
            std::pair< base::Vector3d, base::Vector3d > model_wall2;
            double fitrate_wall2 = ransac(outlier_wall1, iterations, threshold, outlier_wall2, model_wall2);
            fitrate_wall2 = fitrate_wall2 * (double)outlier_wall1.size() / (double)pointCloud.size();
            
            if (fitrate_wall2 > 0.0 && fitrate_wall2 > fit_rate / 4.0)
            {
                // handle two walls
                if (fitrate_wall1 + fitrate_wall2 > fit_rate)
                {
                    double angle = computeAngle(model_wall1, model_wall2);
                    if (angle > M_PI_2 - right_angle_variance && angle < M_PI_2 + right_angle_variance)
                    {
                        best_models.push_back(model_wall1);
                        best_models.push_back(model_wall2);
                        best_error = 1.0 - fitrate_wall1 - fitrate_wall2;
                        return best_error;
                    }
                }
            }
            */
            // handle one wall
            if (fitrate_wall1 > fit_rate)
            {
                best_models.push_back(model_wall1);
                best_error = 1.0 - fitrate_wall1;
            }
        }
    }
    return best_error;
}

/**
 * Implementation of the ransac algorithm for line estimation in a 3d point cloud.
 * 
 * @param pointCloud - the 3d point cloud
 * @param iterations - number of iterations
 * @param threshold - max inlier distance to the model
 * @param outlier_best_model - outliers of the best model
 * @param best_model - best model that could be found
 * @return fit_rate - count inliers of the best model in percent
 */
double ransac(const std::vector< base::Vector3d >& pointCloud, unsigned iterations, double threshold, std::vector< base::Vector3d >& outlier_best_model, std::pair< base::Vector3d, base::Vector3d >& best_model)
{
    srand(time(NULL));
    double best_fit_rate = 0.0;
    
    // check for diverse points
    bool diverse_points = false;
    for(unsigned i = 1; i < pointCloud.size(); i++)
    {
        if(pointCloud[0] != pointCloud[i])
        {
            diverse_points = true;
            break;
        }
    }
    
    if (diverse_points)
    {
        // ransac algorithm
        for(unsigned i = 0; i < iterations; i++)
        {
            base::Vector3d model_p1 = pointCloud[rand() % pointCloud.size()];
            base::Vector3d model_p2;
            do 
            {
                model_p2 = pointCloud[rand() % pointCloud.size()];
            }
            while(model_p1 == model_p2);
            
            std::pair< base::Vector3d, base::Vector3d > model(model_p1, model_p1 - model_p2);
            std::vector< base::Vector3d > consensus_set;
            std::vector< base::Vector3d > outlier;

            double fit = lineFitEvaluation(pointCloud, model, threshold, consensus_set, outlier);
            
            if (fit > best_fit_rate)
            {
                computeLine(consensus_set, best_model);
                outlier_best_model = outlier;
                best_fit_rate = fit;
            }
        }
    }
    return best_fit_rate;
}

double lineFitEvaluation(const std::vector< base::Vector3d >& pointCloud, const std::pair< base::Vector3d, base::Vector3d >& line, const double threshold, 
                         std::vector< base::Vector3d >& inlier, std::vector< base::Vector3d >& outlier)
{
    for(std::vector< base::Vector3d >::const_iterator it = pointCloud.begin(); it != pointCloud.end(); it++)
    {
        if (computeDistance(line, *it) <= threshold)
        {
            inlier.push_back(*it);
        }
        else 
        {
            outlier.push_back(*it);
        }
    }

    return (double)inlier.size() / (double)pointCloud.size();
}

/**
 * Compute the nearest point on the line to the given point.
 *
 * @param line the line
 * @param point reverence point
 * @return the nearest tangential point
 */
base::Vector3d computIntersection(const std::pair< base::Vector3d, base::Vector3d >& line, const base::Vector3d& point)
{
    double lambda = point.x() * line.second.x() + point.y() * line.second.y() + point.z() * line.second.z();
    double x = line.second.x() * line.first.x() + line.second.y() * line.first.y() + line.second.z() * line.first.z();
    double y = line.second.x() * line.second.x() + line.second.y() * line.second.y() + line.second.z() * line.second.z();
    lambda -= x;
    lambda /= y;
    return line.first + (lambda * line.second);
}

/**
 * Compute the distance between two 3d points.
 *
 * @param vec1
 * @param vec2
 * @return the distance in m
 */
double computeDistance(const base::Vector3d& vec1, const base::Vector3d& vec2)
{
    return sqrt(pow(vec1.x()-vec2.x(), 2) + pow(vec1.y()-vec2.y(), 2) + pow(vec1.z()-vec2.z(), 2));
}

/**
 * Compute the shoortest distance between a 3d line and a 3d point. 
 *
 * @param line the line
 * @param point reverence point
 * @return the distance in m
 */
double computeDistance(const std::pair< base::Vector3d, base::Vector3d >& line, const base::Vector3d& point)
{
    base::Vector3d intersection_point = computIntersection(line, point);
    return computeDistance(point, intersection_point);
}

/**
 * Compute the angle between two 3d lines.
 *
 * @param line1
 * @param line2
 * @return the angle in rad
 */
double computeAngle(const std::pair< base::Vector3d, base::Vector3d >& line1, const std::pair< base::Vector3d, base::Vector3d >& line2)
{
    return computeAngle(line1.second, line2.second);
}

/**
 * Compute the angle between two 3d vectors.
 *
 * @param vec1
 * @param vec2
 * @return the angle in rad
 */
double computeAngle(const base::Vector3d& vec1, const base::Vector3d& vec2)
{
    double angle = 0;
    double x = std::abs(vec1.x() * vec2.x() + vec1.y() * vec2.y() + vec1.z() * vec2.z());
    double y = length(vec1) * length(vec2);
    if (y > 0)
    {
        angle = acos(x/y);
        if (angle > M_PI_2) angle = abs(angle - M_PI);
    }
    return angle;
}

/**
 * Compute the length of a vector.
 *
 * @param vec the vector
 * @return the lenght in m
 */
double length(const base::Vector3d& vec)
{
    return sqrt(pow(vec.x(),2) + pow(vec.y(),2) + pow(vec.z(),2));
}

/**
 * Compute the length of a vector.
 *
 * @param vec the vector
 * @return the lenght in m
 */
double length(const base::Vector2d& vec)
{
    return sqrt(pow(vec.x(),2) + pow(vec.y(),2));
}

/**
 * Uses openCV to approximate a line in a given point cloud.
 *
 * @param pointCloud the point cloud
 * @param model the approximated line in hesse normal form
 */
void computeLine(const std::vector< base::Vector3d >& pointCloud, std::pair< base::Vector3d, base::Vector3d >& model)
{
    std::vector<cv::Point3f> cvPoints;
    for(std::vector< base::Vector3d >::const_iterator it = pointCloud.begin(); it != pointCloud.end(); it++)
    {
        cvPoints.push_back(cv::Point3f(it->x(), it->y(), it->z()));
    }

    if(cvPoints.size() >= 2) 
    {
        cv::Vec6f line;
        cv::Mat mat = cv::Mat(cvPoints);
        cv::fitLine(mat, line, CV_DIST_L2, 0, 0.01, 0.01);
        model.first = base::Vector3d(line[3], line[4], line[5]);
        model.second = base::Vector3d(line[0], line[1], line[2]);
        model.first = computIntersection(model, base::Vector3d(0,0,0));
    }
}

bool isInAngularRange(const base::Angle& angle, const base::Angle& left_limit, const base::Angle& right_limit)
{
    bool range_switch = false;
    if(right_limit.rad - left_limit.rad > 0.0)
        range_switch = true;
    
    if((range_switch && (angle < left_limit || angle > right_limit)) ||
        (!range_switch && (angle < left_limit && angle > right_limit)))
        return true;
    else
        return false;
}

}
