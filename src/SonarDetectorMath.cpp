#include "SonarDetectorMath.hpp"
#include <opencv/cv.h>

namespace avalon
{

/**
 * Implementation of the ransac algorithm for line estimation in a 3d point cloud.
 * 
 * @param pointCloud - the 3d point cloud
 * @param iterations - number of iterations
 * @param threshold - max inlier distance to the model
 * @param fit_rate -  min count inliers of the point cloud in percent, so it is a valid model
 * @param best_model - best model that could be found
 * @return best_error, outlier in percent. if returns 1.0, no valid model could be found
 */
double ransac(const std::vector< base::Vector3d >& pointCloud, int iterations, double threshold, double fit_rate, std::pair< base::Vector3d, base::Vector3d >& best_model)
{
    srand(time(NULL));
    double best_error = 1.0;
    
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

            for(std::vector< base::Vector3d >::const_iterator it = pointCloud.begin(); it != pointCloud.end(); it++)
            {
                if (*it == model_p1 || *it == model_p2)
                    continue;
                
                if (computeDistance(model, *it) <= threshold)
                {
                    consensus_set.push_back(*it);
                }
            }
            
            double fit = (double)consensus_set.size() / (double)pointCloud.size();
            if (fit >= fit_rate && 1.0 - fit < best_error)
            {
                computeModel(consensus_set, best_model);
                best_error = 1.0 - fit;
            }
        }
    }
    return best_error;
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