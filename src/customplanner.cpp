#include <points_global_planner/customplanner.h>
#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/costmap_2d.h>
#include <std_msgs/Header.h>

PLUGINLIB_EXPORT_CLASS(points_global_planner::CustomPlanner, nav_core::BaseGlobalPlanner)

namespace points_global_planner {
    CustomPlanner::CustomPlanner() :
        costmap_ros_(NULL), initialized_(false) {
    }

    CustomPlanner::CustomPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
        costmap_ros_(NULL), initialized_(false) {
        initialize(name, costmap_ros);
    }

    CustomPlanner::~CustomPlanner() {
    }

    void CustomPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_) {
            ros::NodeHandle private_nh("~/" + name);
            if (!private_nh.hasParam("points_filename")) {
                ROS_ERROR("[CustomPlanner] The points_filename parameter not set");
                exit(-1);
            }
            private_nh.param<std::string>("points_filename", points_filename_, " ");
            ROS_INFO("[CustomPlanner] Started CustomPlanner with: %s", points_filename_.c_str());
            if (!readPointsFromYAML(points_filename_, points_)) {
                ROS_ERROR("[CustomPlanner] Could not read points from file");
                exit(-1);
            }
            ROS_INFO("[CustomPlanner] Read %d points from file", (int)points_.size());

            costmap_ros_ = costmap_ros;
            global_frame_ = costmap_ros_->getGlobalFrameID();
            planner_ = new navfn::NavfnROS();
            planner_->initialize(name, costmap_ros_);

            double max_distance_;
            private_nh.param<double>("max_distance", max_distance_, 1.0);
            ROS_INFO("[CustomPlanner] Max distance between points: %f", max_distance_);

            generateGraph(points_, max_distance_, graph_, weightMap_);
            ROS_INFO("[CustomPlanner] Generated graph with %d vertices and %d edges", (int)boost::num_vertices(graph_), (int)boost::num_edges(graph_));

            markerPub_ = private_nh.advertise<visualization_msgs::Marker>("points_global_planner", 1);
            planPub_ = private_nh.advertise<nav_msgs::Path>("plantest", 1);
            initialized_ = true;
        }
        else {
        ROS_WARN("[CustomPlanner] This planner has already been initialized... doing nothing");
        }
    }

    bool CustomPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan) {
        if (!initialized_) {
        ROS_ERROR("[CustomPlanner] The planner has not been initialized, please call initialize() to use the planner");
        return false;
        }
        plan.clear();
        if (goal.header.frame_id != global_frame_) {
        ROS_ERROR("[CustomPlanner] This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                    global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
        }
        if (start.header.frame_id != global_frame_) {
        ROS_ERROR("[CustomPlanner] This planner as configured will only accept start in the %s frame, but a start was sent in the %s frame.",
                    global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
        }
        // Call planner
        if (points_.empty()) {
            ROS_ERROR("[CustomPlanner] No points to plan");
            return false;
        }

        int startIdx = nearestPointSearch(points_, start);
        int goalIdx = nearestPointSearch(points_, goal);
        std::vector<Vertex> vertex_path = findShortestPath(graph_, startIdx, goalIdx, weightMap_);
        std::vector<geometry_msgs::PoseStamped> tmp_plan;
        bool success = true;
        success = success && planner_->makePlan(start, points_[vertex_path[0]], tmp_plan);
        plan.insert(plan.end(), tmp_plan.begin(), tmp_plan.end());
        tmp_plan.clear();
        for (size_t i = 0; i < vertex_path.size() - 1; ++i) {
            success = success && planner_->makePlan(points_[vertex_path[i]], points_[vertex_path[i + 1]], tmp_plan);
            plan.insert(plan.end(), tmp_plan.begin(), tmp_plan.end());
            tmp_plan.clear();
        }
        success = success && planner_->makePlan(points_[vertex_path[vertex_path.size() - 1]], goal, tmp_plan);
        plan.insert(plan.end(), tmp_plan.begin(), tmp_plan.end());
        tmp_plan.clear();

        nav_msgs::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = ros::Time::now();
        path.poses = plan;
        planPub_.publish(path);
        return success;
    }

    bool CustomPlanner::readPointsFromYAML(const std::string& filePath, std::vector<geometry_msgs::PoseStamped>& points){
        try {
            YAML::Node yamlNode = YAML::LoadFile(filePath);

            for (YAML::const_iterator it = yamlNode.begin(); it != yamlNode.end(); ++it) {
                const std::string& pointName = it->first.as<std::string>();
                const YAML::Node& pointMap = it->second;

                if (pointMap["position"].IsSequence() && pointMap["frame_id"].IsScalar()) {
                    const YAML::Node& positionNode = pointMap["position"];
                    const std::string& frameId = pointMap["frame_id"].as<std::string>();

                    geometry_msgs::PoseStamped poseStamped;
                    poseStamped.header.frame_id = frameId;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.pose.position.x = positionNode[0].as<double>();
                    poseStamped.pose.position.y = positionNode[1].as<double>();
                    poseStamped.pose.position.z = positionNode[2].as<double>();
                    poseStamped.pose.orientation.w = 1.0;

                    points.push_back(poseStamped);
                }
                else {
                    ROS_ERROR_STREAM("[CustomPlanner] Error reading YAML file: " << filePath << ". The point " << pointName << " is not well formed.");
                }
            }
        }catch (const YAML::Exception& e) {
            ROS_ERROR_STREAM("[CustomPlanner] Error reading YAML file: " << e.what());
            return false;
        }

        return true;
    }

    int CustomPlanner::nearestPointSearch(const std::vector<geometry_msgs::PoseStamped>& points, const geometry_msgs::PoseStamped& pose){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointXYZ searchPoint;
        searchPoint.x = pose.pose.position.x;
        searchPoint.y = pose.pose.position.y;
        searchPoint.z = pose.pose.position.z;
        cloud->width = points.size();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);
        for (size_t i = 0; i < points.size(); ++i) {
            cloud->points[i].x = points[i].pose.position.x;
            cloud->points[i].y = points[i].pose.position.y;
            cloud->points[i].z = points[i].pose.position.z;
        }
        kdtree.setInputCloud(cloud);
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            return pointIdxNKNSearch[0];
        }
        else {
            ROS_ERROR("[CustomPlanner] No nearest point found");
            return -1;
        }
     }

    double CustomPlanner::calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
        double dx = pose1.pose.position.x - pose2.pose.position.x;
        double dy = pose1.pose.position.y - pose2.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void CustomPlanner::generateGraph(const std::vector<geometry_msgs::PoseStamped>& points, double max_distance,
                                        Graph& graph, WeightMap& weightMap) {
        weightMap = boost::get(boost::edge_weight, graph);

        std::vector<Vertex> vertices;

        for (const auto& point : points_) {
            Vertex v = boost::add_vertex(graph);
            vertices.push_back(v);
        }

        for (size_t i = 0; i < vertices.size(); ++i) {
            for (size_t j = i + 1; j < vertices.size(); ++j) {
                double distance = calculateDistance(points_[i], points_[j]);
                if (distance <= max_distance) {
                    Edge e;
                    bool added;
                    boost::tie(e, added) = boost::add_edge(vertices[i], vertices[j], graph);
                    if (added) {
                        weightMap[e] = distance;
                    }
                }
            }
        }

    }

    std::vector<Vertex> CustomPlanner::findShortestPath(Graph& graph, int startIdx, int goalIdx, WeightMap& weightMap){
        Vertex startVertex = boost::vertex(startIdx, graph);
        Vertex goalVertex = boost::vertex(goalIdx, graph);
        std::vector<Vertex> predecessors(boost::num_vertices(graph));
        std::vector<double> distances(boost::num_vertices(graph));

        boost::dijkstra_shortest_paths(graph, startVertex,
                                    boost::distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, graph)))
                                        .predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph)))
                                        .weight_map(weightMap));

        std::vector<Vertex> path;
        for (Vertex v = goalVertex; v != startVertex; v = predecessors[v]) {
            path.push_back(v);
        }
        path.push_back(startVertex);
        std::reverse(path.begin(), path.end());
        publishMarkers(path);
        return path;
    }

    void CustomPlanner::publishMarkers(const std::vector<Vertex>& indexes){
        visualization_msgs::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "points_global_planner";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (const auto& index : indexes) {
            geometry_msgs::Point point;
            point.x = points_[index].pose.position.x;
            point.y = points_[index].pose.position.y;
            point.z = points_[index].pose.position.z;
            marker.points.push_back(point);
        }

        markerPub_.publish(marker);
    }

};