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

class PRM{
public:
    explicit PRM(ros::NodeHandle& nodeHandle, bool unknown, int threshold, double radius);
    void plan();
    nav_msgs::Path getPath();
    nav_msgs::OccupancyGrid::Ptr getInflatedMap();

private:
    ros::NodeHandle nh;

    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid::Ptr inflated_map;
    geometry_msgs::Pose amcl_pose;
    nav_msgs::Path ros_solution_path;

    ros::Subscriber map_subscriber;
    ros::Subscriber amcl_pose_subscriber;

    bool map_received;
    bool pose_received;
    bool unknown_okay;

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
