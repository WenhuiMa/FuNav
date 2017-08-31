#ifndef COSTMAP_ROS_H_MWH_20170831
#define COSTMAP_ROS_H_MWH_20170831

#include"layered_costmap.h"
#include"layer.h"
#include"costmap_publisher.h"
#include"footprint.h"
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <pluginlib/class_loader.h>

class SuperValue : public XmlRpc::XmlRpcValue
{
public:
  void setStruct(XmlRpc::XmlRpcValue::ValueStruct* a)
  {
    _type = TypeStruct;
    _value.asStruct = new XmlRpc::XmlRpcValue::ValueStruct(*a);
  }
  void setArray(XmlRpc::XmlRpcValue::ValueArray* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace costmap
{

    class CostmapROS
    {
    public:
        CostmapROS(std::string name, tf::TransformListener& tf);
        ~CostmapROS();
        void start();
        void stop();
        void pause();
        void resume();
        void updateMap();
        void resetLayers();
        bool isCurrent()
        {
            return layered_costmap_->isCurrent();
        }

        bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;

        Costmap* getCostmap()
        {
          return layered_costmap_->getCostmap();
        }

        std::string getGlobalFrameID()
        {
          return global_frame_;
        }

        std::string getBaseFrameID()
        {
          return robot_base_frame_;
        }

        LayeredCostmap* getLayeredCostmap()
        {
          return layered_costmap_;
        }

        geometry_msgs::Polygon getRobotFootprintPolygon()
        {
          return costmap::toPolygon(padded_footprint_);
        }

        std::vector<geometry_msgs::Point> getRobotFootprint()
        {
          return padded_footprint_;
        }

        std::vector<geometry_msgs::Point> getUnpaddedRobotFootprint()
        {
          return unpadded_footprint_;
        }

        void getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const;

        void setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points);

        void setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint);

    protected:
        LayeredCostmap* layered_costmap_;
        std::string name_;
        tf::TransformListener& tf_;
        std::string global_frame_; 
        std::string robot_base_frame_;
        double transform_tolerance_;

    private:
        //void readFootprintFromConfig(const costmap::CostmapConfig &new_config,
        //    const costmap::CostmapConfig &old_config);
        void resetOldParameters(ros::NodeHandle& nh);
        void movementCB(const ros::TimerEvent &event);
        void mapUpdateLoop(double frequency);
        bool map_update_thread_shutdown_;
        bool stop_updates_, initialized_, stopped_, robot_stopped_;
        boost::thread* map_update_thread_;
        ros::Timer timer_;
        ros::Time last_publish_;
        ros::Duration publish_cycle;
        pluginlib::ClassLoader<Layer> plugin_loader_;
        tf::Stamped<tf::Pose> old_pose_;
        CostmapPublisher* publisher_;

        ros::Subscriber footprint_sub_;
        ros::Publisher footprint_pub_;
        bool got_footprint_;
        std::vector<geometry_msgs::Point> unpadded_footprint_;
        std::vector<geometry_msgs::Point> padded_footprint_;
        float footprint_padding_;
    };

}



#endif