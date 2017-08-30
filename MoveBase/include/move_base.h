#ifndef MOVE_BASE_H_MWH_20170829
#define MOVE_BASE_H_MWH_20170829

#include <vector>
#include <string>

#include "ros/ros.h"

#include "actionlib/server/simple_action_server.h"
#include "move_base_msgs/MoveBaseAction.h"

#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
namespace move_base
{
    typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;


    class MoveBase
    {
    public:
        MoveBase(tf::TransformListener& tf);
        virtual ~MoveBase();
        bool executeCycle(geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan);
    private:
        bool getPlan();

        void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);

        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

        tf::TransformListener& tf_;
        MoveBaseActionServer* as_;

        tf::Stamped<tf::Pose> global_pose_;

        double controller_frequency_, inscribed_radius_, circumscribed_radius_;
        ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_;
        std::vector<geometry_msgs::PoseStamped>* latest_plan_;
        std::vector<geometry_msgs::PoseStamped>* controller_plan_;

    };

};

#endif
