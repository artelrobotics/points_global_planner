#ifndef _CUSTOMPLANNER_H_
#define _CUSTOMPLANNER_H_

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <yaml-cpp/yaml.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <cmath>
#include <fstream>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <tf/tf.h>

typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, EdgeWeightProperty> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::property_map<Graph, boost::edge_weight_t>::type WeightMap;

namespace points_global_planner {
  class CustomPlanner : public nav_core::BaseGlobalPlanner {
    public:
      CustomPlanner();
      CustomPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      ~CustomPlanner();
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);
    protected:
      bool readPointsFromYAML(const std::string& filePath, std::vector<geometry_msgs::PoseStamped>& points);
      int nearestPointSearch(const std::vector<geometry_msgs::PoseStamped>& points,
                              const geometry_msgs::PoseStamped& pose);
      costmap_2d::Costmap2DROS* costmap_ros_;
      std::string global_frame_;
      bool initialized_;
      navfn::NavfnROS* planner_;
      std::vector<geometry_msgs::PoseStamped> points_;
      std::string points_filename_;

      double max_distance_;
      Graph graph_;
      WeightMap weightMap_;
      ros::Publisher markerPub_;
      ros::Publisher planPub_;
      double calculateDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);
      void generateGraph(const std::vector<geometry_msgs::PoseStamped>& points, double max_distance,
                                        Graph& graph, WeightMap& weightMap);
      std::vector<Vertex> findShortestPath(Graph& graph, int startIdx, int goalIdx, WeightMap& weightMap);
      void publishMarkers(const std::vector<Vertex>& indexes);
  };
};

#endif
