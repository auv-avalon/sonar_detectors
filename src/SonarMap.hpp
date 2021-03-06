#ifndef SONAR_MAP_HPP_
#define SONAR_MAP_HPP_

#include <list>
#include <vector>
#include <base/time.h>
#include <stdint.h>

namespace sonar_detectors
{

template <class T>
class SonarMap
{
public:
    SonarMap()
    {
        angle_it = angles.end();
        feature_it = features.end();
        featureTimeout = 8000000; // 8s
        lastTimeoutCheck = base::Time::now().toMicroseconds();
    };
    
    void addFeature(const T feature, const double angle, const base::Time time)
    {
        // inital entry
        if(angles.empty())
        {
            insertBeforeCurrentEntry(feature, angle, time);
        }
        else
        {
            // handle switch between -PI and PI
            if (fabs(angle_it->first - angle) > M_PI)
            {
                if (angle >= 0) 
                {
                    angle_it = angles.end();
                    feature_it = features.end();
                    angle_it--;
                    feature_it--;
                }
                else 
                {
                    angle_it = angles.begin();
                    feature_it = features.begin();
                }
            }
            // add features in the same angle
            if(angle == angle_it->first)
            {
                removeCurrentEntry();
                insertBeforeCurrentEntry(feature, angle, time);
            }
            // add features after the last angle
            else if(angle > angle_it->first)
            {
                angle_it++;
                feature_it++;
                while(angle_it != angles.end() && angle >= angle_it->first)
                {
                    removeCurrentEntry();
                }
                insertBeforeCurrentEntry(feature, angle, time);
            }
            // add features before the last angle
            else
            {
                angle_it--;
                feature_it--;
                while(angle_it != angles.end() && angle_it != angles.begin() && angle <= angle_it->first)
                {
                    removeCurrentEntry();
                    feature_it--;
                    angle_it--;
                }
                angle_it++;
                feature_it++;
                insertBeforeCurrentEntry(feature, angle, time);
            }
        }
        // check every second for outdated features
        if((base::Time::now().toMicroseconds() - lastTimeoutCheck) >= 1000000)
        {
            removeOutdatedFeatures();
            lastTimeoutCheck = base::Time::now().toMicroseconds();
        }
    };
    
    void clean()
    {
        features.clear();
        angles.clear();
        angle_it = angles.end();
        feature_it = features.end();
    };
    
    void setFeatureTimeout(int64_t milliseconds)
    {
        featureTimeout = 1000 * milliseconds;
    };
    
    std::list<T> getFeatureList() const
    {
        return features;
    };
    
    void getSubFeatureVector(std::vector<T>& subvector, double start_angle = M_PI, double end_angle = -M_PI) const
    {
        subvector.clear();
        bool range_switch = false;
        if(end_angle - start_angle > 0)
            range_switch = true;
        
        typename std::list<T>::const_iterator f_it = features.begin();
        for(std::list< std::pair< double, base::Time > >::const_iterator a_it = angles.begin(); a_it != angles.end(); a_it++, f_it++)
        {
            if((range_switch && (a_it->first <= start_angle || a_it->first >= end_angle)) ||
               (!range_switch && (a_it->first <= start_angle && a_it->first >= end_angle)))
            {
                subvector.push_back(*f_it);
            }
        }
    };
    
    std::list<T> *getFeatureListPtr()
    {
        return &features;
    };
    
protected:
    void insertBeforeCurrentEntry(const T &feature, const double &angle, const base::Time& time)
    {
        feature_it = features.insert(feature_it, feature);
        angle_it = angles.insert(angle_it, std::pair< double , base::Time >(angle, time));
    };
    
    void removeCurrentEntry()
    {
        angle_it = angles.erase(angle_it);
        feature_it = features.erase(feature_it);
    };
    
    void removeOutdatedFeatures()
    {
        if(!angles.empty() && featureTimeout > 0)
        {
            typename std::list<T>::iterator f_it = features.begin();
            std::list< std::pair< double , base::Time > >::iterator a_it = angles.begin();
            base::Time newest_timestamp = a_it->second;
            while(a_it != angles.end())
            {
                if((newest_timestamp.toMicroseconds() - a_it->second.toMicroseconds()) > featureTimeout)
                {
                    a_it = angles.erase(a_it);
                    f_it = features.erase(f_it);
                }
                else
                {
                    a_it++;
                    f_it++;
                }
            }
            if(angles.empty())
                clean();
        }
    }
    
    typename std::list<T> features;     // sonar features
    std::list< std::pair< double, base::Time > > angles;    // angle and time
    std::list< std::pair< double, base::Time > >::iterator angle_it;
    typename std::list<T>::iterator feature_it;
    int64_t featureTimeout;
    int64_t lastTimeoutCheck;
};
    
};

#endif
