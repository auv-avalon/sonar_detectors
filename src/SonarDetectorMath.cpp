#include "SonarDetectorMath.hpp"
#include <opencv/cv.h>

namespace avalon
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
double wallRansac(const std::vector< base::Vector3d >& pointCloud, int iterations, double threshold, double fit_rate, std::vector< std::pair< base::Vector3d, base::Vector3d > >& best_models)
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
double ransac(const std::vector< base::Vector3d >& pointCloud, int iterations, double threshold, std::vector< base::Vector3d >& outlier_best_model, std::pair< base::Vector3d, base::Vector3d >& best_model)
{
    srand(time(NULL));
    double best_fit_rate = 0.0;
    
    if (pointCloud.size() >= 2)
    {
        for(int i = 0; i < iterations; i++)
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
            consensus_set.push_back(model_p1);
            consensus_set.push_back(model_p2);
            std::vector< base::Vector3d > outlier;

            for(std::vector< base::Vector3d >::const_iterator it = pointCloud.begin(); it != pointCloud.end(); it++)
            {
                if (*it == model_p1 || *it == model_p2)
                    continue;
                
                if (computeDistance(model, *it) <= threshold)
                {
                    consensus_set.push_back(*it);
                }
                else 
                {
                    outlier.push_back(*it);
                }
            }
            
            double fit = (double)consensus_set.size() / (double)pointCloud.size();
            if (fit > best_fit_rate)
            {
                computeModel(consensus_set, best_model);
                outlier_best_model = outlier;
                best_fit_rate = fit;
            }
        }
    }
    return best_fit_rate;
}

base::Vector3d computIntersection(const std::pair< base::Vector3d, base::Vector3d >& line, const base::Vector3d& point)
{
    double lambda = point.x() * line.second.x() + point.y() * line.second.y() + point.z() * line.second.z();
    double x = line.second.x() * line.first.x() + line.second.y() * line.first.y() + line.second.z() * line.first.z();
    double y = line.second.x() * line.second.x() + line.second.y() * line.second.y() + line.second.z() * line.second.z();
    lambda -= x;
    lambda /= y;
    return line.first + (lambda * line.second);
}

double computeDistance(const base::Vector3d& vec1, const base::Vector3d& vec2)
{
    return sqrt(pow(vec1.x()-vec2.x(), 2) + pow(vec1.y()-vec2.y(), 2) + pow(vec1.z()-vec2.z(), 2));
}

double computeDistance(const std::pair< base::Vector3d, base::Vector3d >& line, const base::Vector3d& point)
{
    base::Vector3d intersection_point = computIntersection(line, point);
    return computeDistance(point, intersection_point);
}

double computeAngle(const std::pair< base::Vector3d, base::Vector3d >& line1, const std::pair< base::Vector3d, base::Vector3d >& line2)
{
    double angle = 0;
    base::Vector3d r1 = line1.second;
    base::Vector3d r2 = line2.second;
    double x = std::abs(r1.x() * r2.x() + r1.y() * r2.y() + r1.z() * r2.z());
    double y = length(r1) * length(r2);
    if (y > 0)
    {
        angle = acos(x/y);
        if (angle > M_PI_2) angle = abs(angle - M_PI);
    }
    return angle;
}

double length(const base::Vector3d& vec)
{
    return sqrt(pow(vec.x(),2) + pow(vec.y(),2) + pow(vec.z(),2));
}

void computeModel(const std::vector< base::Vector3d >& pointCloud, std::pair< base::Vector3d, base::Vector3d >& model)
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
    }
}

}