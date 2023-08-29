#include <points_global_planner/create_points.h>

CreatePoints::CreatePoints(ros::NodeHandle& nh) : nh_(nh) {
    clickedPointSub = nh_.subscribe("/clicked_point", 10, &CreatePoints::clickedPointCallback, this);
    savePointsService = nh_.advertiseService("save_points", &CreatePoints::savePointsToYAML, this);
    deletePointService = nh_.advertiseService("delete_point", &CreatePoints::deletePoint, this);
    visualizePoints = nh_.param<bool>("visualize_points", true);
    if (visualizePoints) {
        markerPub = nh_.advertise<visualization_msgs::Marker>("points_plan", 10);
    }
}

void CreatePoints::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    points.push_back(*msg);
    ROS_INFO("Received point: (%f, %f, %f)", msg->point.x, msg->point.y, msg->point.z);
    if (visualizePoints) {
        publishPointsAsMarkers();
    }
}

bool CreatePoints::savePointsToYAML(map_msgs::SaveMap::Request& req, map_msgs::SaveMap::Response& res) {
    std::string filename = req.filename.data + ".yaml";
    YAML::Emitter out;
    out << YAML::BeginMap;
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        out << YAML::Key << "point_" + std::to_string(i);
        out << YAML::Value;
        out << YAML::BeginMap;
        out << YAML::Key << "position";
        out << YAML::Value << YAML::Flow << YAML::BeginSeq << point.point.x << point.point.y << point.point.z << YAML::EndSeq;
        out << YAML::Key << "frame_id";
        out << YAML::Value << point.header.frame_id;
        out << YAML::EndMap;
    }
    out << YAML::EndMap;

    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << out.c_str();
        outFile.close();
        ROS_INFO("Clicked points saved to %s", filename.c_str());
        return true;
    } else {
        ROS_ERROR("Unable to open file for saving clicked points");
        return false;
    }
}

bool CreatePoints::deletePoint(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
   if (!points.empty()) {
        points.pop_back();
        ROS_INFO("Last point deleted");
        res.success = true;
        res.message = "Last point deleted";
    } else {
        ROS_WARN("No points to delete");
        res.success = false;
        res.message = "No points to delete";
    }
    if (visualizePoints) {
        publishPointsAsMarkers();
    }
    return true;
}

void CreatePoints::publishPointsAsMarkers() {
    marker.header.stamp = ros::Time();
    marker.ns = "points_global_planner";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.clear();
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    for (const auto& point : points) {
        marker.header.frame_id = point.header.frame_id;
        marker.points.push_back(point.point);
    }
    markerPub.publish(marker);
}
