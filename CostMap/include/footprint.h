#ifndef FOOTPRINT_H_MWH_20170831
#define FOOTPRINT_H_MWH_20170831

#include<ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

namespace costmap
{
    void calculateMinAndMaxDistances(const std::vector<geometry_msgs::Point>& footprint,
        double& min_dist, double& max_dist);
    geometry_msgs::Point toPoint(geometry_msgs::Point32 pt);
    geometry_msgs::Point32 toPoint32(geometry_msgs::Point pt);
    geometry_msgs::Polygon toPolygon(std::vector<geometry_msgs::Point> pts);
    std::vector<geometry_msgs::Point> toPointVector(geometry_msgs::Polygon polygon);
    void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
        std::vector<geometry_msgs::Point>& oriented_footprint);
    void transformFootprint(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
            geometry_msgs::PolygonStamped & oriented_footprint);
    void padFootprint(std::vector<geometry_msgs::Point>& footprint, double padding);
    std::vector<geometry_msgs::Point> makeFootprintFromRadius(double radius);
    bool makeFootprintFromString(const std::string& footprint_string, std::vector<geometry_msgs::Point>& footprint);
    std::vector<geometry_msgs::Point> makeFootprintFromParams(ros::NodeHandle& nh);
    std::vector<geometry_msgs::Point> makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
        const std::string& full_param_name);
    void writeFootprintToParam(ros::NodeHandle& nh, const std::vector<geometry_msgs::Point>& footprint);

}

#endif