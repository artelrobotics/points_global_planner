#ifndef _CREATEPOINTS_H_
#define _CREATEPOINTS_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <map_msgs/SaveMap.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>

class CreatePoints {
public:
    CreatePoints(ros::NodeHandle& nh);

private:
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    bool savePointsToYAML(map_msgs::SaveMap::Request& req, map_msgs::SaveMap::Response& res);
    bool deletePoint(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void publishPointsAsMarkers();
    ros::NodeHandle nh_;
    ros::Subscriber clickedPointSub;
    ros::ServiceServer savePointsService;
    ros::ServiceServer deletePointService;
    std::vector<geometry_msgs::PointStamped> points;
    visualization_msgs::Marker marker;
    ros::Publisher markerPub;
    bool visualizePoints;
};

#endif // _CREATEPOINTS_H_
