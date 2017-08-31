#ifndef OBSTACLE_LAYER_H_MWH_20170831
#define OBSTACLE_LAYER_H_MWH_20170831

#include<ros/ros.h>
#include"costmap.h"
#include"layered_costmap.h"
#include"observation_buffer.h"

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>

#include "footprint.h"

namespace costmap
{
    class ObstacleLayer:public CostmapLayer
    {
    public:
        ObstacleLayer()
        {
            costmap_ = NULL;//from parent class
        }
        virtual ~ObstacleLayer();
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, 
                                  double robot_y, 
                                  double robot_yaw, 
                                  double* min_x, 
                                  double* min_y,
                                  double* max_x, 
                                  double* max_y);

        virtual void updateCosts(costmap::Costmap& master_grid, 
                                int min_i, int min_j, int max_i, int max_j);

        virtual void activate();
        virtual void deactivate();
        virtual void reset();  

        void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
            const boost::shared_ptr<costmap::ObservationBuffer>& buffer);     

        void laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& message,
                const boost::shared_ptr<ObservationBuffer>& buffer);

        void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                    const boost::shared_ptr<costmap::ObservationBuffer>& buffer);

        void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                        const boost::shared_ptr<costmap::ObservationBuffer>& buffer);
        void addStaticObservation(costmap::Observation& obs, bool marking, bool clearing);
        void clearStaticObservations(bool marking, bool clearing);   
        
    protected:
        bool getMarkingObservations(std::vector<costmap::Observation>& marking_observations) const;
        bool getClearingObservations(std::vector<costmap::Observation>& clearing_observations) const;
        
        virtual void raytraceFreespace(const costmap::Observation& clearing_observation, double* min_x, double* min_y,
            double* max_x, double* max_y);
        
        void updateRaytraceBounds(double ox, double oy, double wx, double wy, double range, double* min_x, double* min_y,
                double* max_x, double* max_y);
        
        std::vector<geometry_msgs::Point> transformed_footprint_;
        bool footprint_clearing_enabled_;
        void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
            double* max_x, double* max_y);
        std::string global_frame_;
        double max_obstacle_height_;
        laser_geometry::LaserProjection projector_;

        std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
        std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
        std::vector<boost::shared_ptr<costmap::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
        std::vector<boost::shared_ptr<costmap::ObservationBuffer> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
        std::vector<boost::shared_ptr<costmap::ObservationBuffer> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles
      
        std::vector<costmap::Observation> static_clearing_observations_, static_marking_observations_;

        bool rolling_window_;
        int combination_method_;
  
    };
}

#endif