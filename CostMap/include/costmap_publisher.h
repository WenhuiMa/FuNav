#ifndef COSTMAP_PUBLISHER_H_MWH_20170831
#define COSTMAP_PUBLISHER_H_MWH_20170831

#include<ros/ros.h>
#include"costmap.h"
#include<nav_msgs/OccupancyGrid.h>
#include<map_msgs/OccupancyGridUpdate.h>
#include<tf/transform_datatypes.h>

namespace costmap
{
    class CostmapPublisher
    {
    public:
        CostmapPublisher(ros::NodeHandle* ros_node,
                         Costmap* costmap, 
                         std::string global_frame,
                         std::string topic_name,
                         bool always_send_full_costmap = false);
        ~CostmapPublisher();
        void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
        {
          x0_ = std::min(x0, x0_);
          xn_ = std::max(xn, xn_);
          y0_ = std::min(y0, y0_);
          yn_ = std::max(yn, yn_);
        }

        void publishCostmap();

        bool active()
        {
          return active_;
        }

    private:
        void prepareGrid();
        void onNewSubscription(const ros::SingleSubscriberPublisher& pub);

        ros::NodeHandle* node;
        Costmap* costmap_;
        std::string global_frame_;
        unsigned int x0_,xn_,y0_,yn_;
        double saved_origin_x_, saved_origin_y_;
        bool active_;
        bool always_send_full_costmap_;
        ros::Publisher costmap_pub_;
        ros::Publisher costmap_update_pub_;
        nav_msgs::OccupancyGrid grid_;
        static char* cost_translation_table_;

    };
}

#endif