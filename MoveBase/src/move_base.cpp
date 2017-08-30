#include "move_base.h"
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "geometry_msgs/Twist.h"

namespace move_base
{
    MoveBase::MoveBase(tf::TransformListener& tf):
    tf_(tf),
    as_(NULL),
    latest_plan_(NULL), 
    controller_plan_(NULL)
    {
        as_ = new MoveBaseActionServer(ros::NodeHandle(), 
                                       "move_base", 
                                       boost::bind(&MoveBase::executeCb,this, _1), 
                                       false);
        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

        ros::NodeHandle simple_nh("move_base_simple");

    }

    MoveBase::~MoveBase(){}

    
    void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
    {
        ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
        move_base_msgs::MoveBaseActionGoal action_goal;
        action_goal.header.stamp = ros::Time::now();
        action_goal.goal.target_pose = *goal;
    
        action_goal_pub_.publish(action_goal);
    }

    void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
    {

    }

    bool MoveBase::getPlan()
    {
        
    }

    void MoveBase::executeCycle(geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan)
    {
        1.get robot current pose
        2.action server publish the feedback
        3.check if stucked or deflected
        4.check the observation(sensordata)
        5.if(got plan)...
        6.switch the state:
            a).waiting for task
            b).controlling
            c).hault
    }

};