#ifndef OBSERVATION_H_MWH_20170831
#define OBSERVATION_H_MWH_20170831

#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace costmap
{
    class Observation
    {
    public:
        Observation():cloud_(new pcl::PointCloud<pcl::PointXYZ>()), 
                             obstacle_range_(0.0),
                             raytrace_range_(0.0){}
        virtual ~Observation()
        {
            delete cloud_;
        }
        
        Observation(geometry_msgs::Point& origin, 
                    pcl::PointCloud<pcl::PointXYZ> cloud,
                    double obstacle_range, 
                    double raytrace_range) :origin_(origin), 
                                            cloud_(new pcl::PointCloud<pcl::PointXYZ>(cloud)),
                                            obstacle_range_(obstacle_range), 
                                            raytrace_range_(raytrace_range)
        {
        }
        
        Observation(const Observation& obs) :origin_(obs.origin_), 
                                             cloud_(new pcl::PointCloud<pcl::PointXYZ>(*(obs.cloud_))),
                                             obstacle_range_(obs.obstacle_range_), 
                                             raytrace_range_(obs.raytrace_range_)
        {
        }

        Observation(pcl::PointCloud<pcl::PointXYZ> cloud, 
                    double obstacle_range) :cloud_(new pcl::PointCloud<pcl::PointXYZ>(cloud)), 
                                            obstacle_range_(obstacle_range), 
                                            raytrace_range_(0.0)
        {
        }

        geometry_msgs::Point origin_;
        pcl::PointCloud<pcl::PointXYZ>* cloud_;
        double obstacle_range_, raytrace_range_;

    };
}
#endif