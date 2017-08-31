#ifndef OBSERVATION_BUFFER_H_MWH_20170831
#define OBSERVATION_BUFFER_H_MWH_20170831

#include <vector>
#include <list>
#include <string>
#include <ros/time.h>
#include "observation.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread.hpp>

namespace costmap
{
    class ObservationBuffer
    {
    public:
        ObservationBuffer(std::string topic_name, 
                          double observation_keep_time, 
                          double expected_update_rate,
                          double min_obstacle_height, 
                          double max_obstacle_height, 
                          double obstacle_range,
                          double raytrace_range, 
                          tf::TransformListener& tf, 
                          std::string global_frame,
                          std::string sensor_frame,
                          double tf_tolerance);
        
        ~ObservationBuffer();

        bool setGlobalFrame(const std::string new_global_frame);

        void bufferCloud(const sensor_msgs::PointCloud2& cloud);
        void bufferCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

        void getObservations(std::vector<Observation>& observations);

        bool isCurrent() const;
        inline void lock()
        {
          lock_.lock();
        }

        inline void unlock()
        {
          lock_.unlock();
        }

        void resetLastUpdated();

    private:
        void purgeStaleObservations();

        tf::TransformListener& tf_;
        const ros::Duration observation_keep_time_;
        const ros::Duration expected_update_rate_;
        ros::Time last_updated_;
        std::string global_frame_;
        std::string sensor_frame_;
        std::list<Observation> observation_list_;
        std::string topic_name_;
        double min_obstacle_height_, max_obstacle_height_;
        boost::recursive_mutex lock_;  // A lock for accessing data in callbacks safely
        double obstacle_range_, raytrace_range_;
        double tf_tolerance_;

    };
}



#endif 