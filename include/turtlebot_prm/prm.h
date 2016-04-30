//
// Created by matt on 4/29/16.
//

#ifndef TURTLEBOT_PRM_PRM_H
#define TURTLEBOT_PRM_PRM_H

#include <ompl/base/State.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ompl/geometric/PathGeometric.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

class PRM : public nav_core::BaseGlobalPlanner {
public:
    PRM();
    PRM(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    explicit PRM(ros::NodeHandle& nodeHandle, bool unknown, int threshold, double radius);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void makePlan();
    bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
    nav_msgs::Path getPath();
    nav_msgs::OccupancyGrid::Ptr getInflatedMap();

private:
    ros::NodeHandle nh;

    tf::TransformListener tf;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid::Ptr inflated_map;
    geometry_msgs::Pose amcl_pose;
    nav_msgs::Path ros_solution_path;

    ros::Subscriber map_subscriber;
    ros::Subscriber amcl_pose_subscriber;

    costmap_2d::Costmap2DROS costmap;
    bool map_received;
    bool pose_received;
    bool unknown_okay;
    bool initialized;

    int occupied_threshold;

    double min_x, max_x;
    double min_y, max_y;
    double robot_radius;

    void initializeSubscribers();
    void mapCallback(nav_msgs::OccupancyGrid new_map);
    void amclPoseCallback(geometry_msgs::PoseWithCovarianceStamped new_pose);
    bool isStateValid(const ompl::base::State *state);
    nav_msgs::Path omplPathToRosPath(ompl::geometric::PathGeometric ompl_path);
};

#endif //TURTLEBOT_PRM_PRM_H
