#ifndef STATIC_LAYER_H_MWH_20170831
#define STATIC_LAYER_H_MWH_20170831

#include <ros/ros.h>
#include"costmap_layer.h"
#include"layered_costmap.h"
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

namespace costmap
{
    class StaticLayer:public CostmapLayer
    {
    public:
        StaticLayer();
        virtual ~StaticLayer();
        virtual void onInitialize();
        virtual void activate();
        virtual void deactivate();
        virtual void reset();

        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
            double* max_x, double* max_y);

        virtual void updateCosts(costmap::Costmap& master_grid, int min_i, int min_j, int max_i, int max_j);
            
        virtual void matchSize();
    private:
        void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
        void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
        unsigned char interpretValue(unsigned char value);
        std::string global_frame_; 
        std::string map_frame_;  
        bool subscribe_to_updates_;
        bool map_received_;
        bool has_updated_data_;
        unsigned int x_, y_, width_, height_;
        bool track_unknown_space_;
        bool use_maximum_;
        bool first_map_only_;   
        bool trinary_costmap_;
        ros::Subscriber map_sub_, map_update_sub_;
      
        unsigned char lethal_threshold_, unknown_cost_value_;


    };
}
#endif