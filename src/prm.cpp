//
// Created by matt on 4/29/16.
// Adapted from the OMPL demo file RigidBodyPlanning.cpp
//

// TODO see about expanding this w/ controls

#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <turtlebot_prm/prm.h>
#include <nav_msgs/OccupancyGrid.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <geometry_msgs/Polygon.h>
#include <limits>
#include <occupancy_grid_utils/shortest_path.h>

PRM::PRM(ros::NodeHandle& nodeHandle, bool unknown = false, int threshold = 50, double radius = 0.2) :
    nh(nodeHandle) {
    map_received = false;
    pose_received = false;
    robot_radius = radius;
    unknown_okay = unknown;
    occupied_threshold = threshold;
    max_x = std::numeric_limits<int>::min();
    max_y = std::numeric_limits<int>::min();
    min_x = std::numeric_limits<int>::max();
    min_y = std::numeric_limits<int>::max();
    initializeSubscribers();
}

void PRM::initializeSubscribers() {
    map_subscriber = nh.subscribe("map", 1, &PRM::mapCallback, this);
    amcl_pose_subscriber = nh.subscribe("amcl_pose", 1, &PRM::amclPoseCallback, this);
}

void PRM::mapCallback( nav_msgs::OccupancyGrid new_map) {
    // Check to see if we've previously received a map
    if (map_received) {
        ROS_WARN("New map received, this shouldn't happen");
    }

    map = new_map;
    inflated_map = occupancy_grid_utils::inflateObstacles(new_map, robot_radius, unknown_okay); // Save new map that we've got
    map_received = true; // Mark that we've received a map

    // Figure out the x and y limits of our map
    geometry_msgs::Polygon bounds = occupancy_grid_utils::gridPolygon(inflated_map->info);
    for (unsigned i = 0; i < bounds.points.size(); i++) {
        if (bounds.points[i].x > max_x) {
            max_x = bounds.points[i].x;
        }
        if (bounds.points[i].x < min_x) {
            min_x = bounds.points[i].x;
        }
        if (bounds.points[i].y > max_y) {
            max_y = bounds.points[i].y;
        }
        if (bounds.points[i].y < min_y) {
            min_y = bounds.points[i].y;
        }
    }
}

void PRM::amclPoseCallback(geometry_msgs::PoseWithCovarianceStamped new_pose) {
    amcl_pose = new_pose.pose.pose;
    pose_received = true;
}

bool PRM::isStateValid(const ompl::base::State *state) {
    // First cast abstract state into expected SE2 state
    const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();

    // Get first component of state and cast into real vector state space
    //const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // Don't need this because all rotations are valid
    // Get second component of state and cast into SO(2) space
    //const ompl::base::SO2StateSpace::StateType *rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1);

    geometry_msgs::Point point;
    point.x = se2state->getX();
    point.y = se2state->getY();
    ROS_DEBUG_STREAM("Checking validity of state ( " << point.x << ", " << point.y <<")");

    if (occupancy_grid_utils::withinBounds(inflated_map->info, point)) { // First check if we're in bounds
        ROS_DEBUG("State is in bounds... now checking occupancy...");
        int index = occupancy_grid_utils::pointIndex(inflated_map->info, point);
        int occupied_status = inflated_map->data[index];

        if (unknown_okay && occupied_status == -1) { // Are unkown cells valid states?
            ROS_DEBUG("State occupancy is unknown and unknown_okay has been set to true");
            return true;
        }
        else if (inflated_map->data[index] < occupied_threshold) {
            ROS_DEBUG("State is not occupied");
            return true;
        }
        else {
            ROS_DEBUG("State is occupied");
        }
    }
    else {
        ROS_DEBUG("State is not in bounds");
    }

    return false;
}

nav_msgs::Path PRM::omplPathToRosPath(ompl::geometric::PathGeometric ompl_path) {
    // Create ROS path message that will be returned
    nav_msgs::Path ros_path;
    ros_path.header.frame_id = "map";

    // Extract states from the path
    std::vector<ompl::base::State*> ompl_states = ompl_path.getStates();

    // Convert each state in a ros-compatible Pose
    for (unsigned i = 0; i < ompl_states.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = ompl_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getX();
        pose.pose.position.y = ompl_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getY();
        ros_path.poses.push_back(pose);
    }

    return ros_path;
}

nav_msgs::Path PRM::getPath() {
    return ros_solution_path;
}

nav_msgs::OccupancyGrid::Ptr PRM::getInflatedMap(){
    return inflated_map;
}

void PRM::plan() {
    // Construct the space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

    ROS_DEBUG("Waiting for a map...");
    while (!map_received) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_DEBUG("Got a map!");

    // Set bounds for the R^2 part of SE(2)
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, min_x);
    bounds.setHigh(0, max_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(1, max_y);
    ROS_DEBUG_STREAM("Set X bounds of map to " << min_x << " -> " << max_x);
    ROS_DEBUG_STREAM("Set Y bounds of map to " << min_y << " -> " << max_y);

    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // Define a simple setup class
    ompl::geometric::SimpleSetup simple_setup(space);
    // TODO make this use a PRM

    // Set state validity checking for this space
    simple_setup.setStateValidityChecker(boost::bind(&PRM::isStateValid, this, _1));

    ROS_DEBUG("Waiting for a pose from AMCL...");
    while (!pose_received) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_DEBUG("Got a pose!");

    // Create start state
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::SE2StateSpace::StateType>()->setX(amcl_pose.position.x);
    start->as<ompl::base::SE2StateSpace::StateType>()->setY(amcl_pose.position.y);
    // start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(pose.orientation.); Don't need this right now
    ROS_DEBUG_STREAM("Set start pose to ( " << amcl_pose.position.x << ", " << amcl_pose.position.y << " )");

    // Create goal state
    ompl::base::ScopedState<> goal(space);
    //goal.random(); // TODO this should be taken interactively from the user
    goal->as<ompl::base::SE2StateSpace::StateType>()->setX(-3);
    goal->as<ompl::base::SE2StateSpace::StateType>()->setY(-4);

    // Set the start and goal state
    simple_setup.setStartAndGoalStates(start, goal);

    // Optional, get more output
    simple_setup.setup();
    simple_setup.print();

    // Attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = simple_setup.solve(1.0);

    if (solved) {
        ROS_INFO("Found solution:");
        simple_setup.simplifySolution();
        ompl::geometric::PathGeometric ompl_solution_path = simple_setup.getSolutionPath();
        ompl_solution_path.print(std::cout); // TODO see if this can be switched to ROS_INFO
        ros_solution_path = omplPathToRosPath(ompl_solution_path);
    }
    else {
        ROS_WARN("No solution found");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "prm");
    ros::NodeHandle nh;
    ROS_INFO("Planning for motion with OMPL");
    PRM prm(nh);
    prm.plan();

    ros::Publisher solution_path_publisher = nh.advertise<nav_msgs::Path>("solution_path", 1);
    nav_msgs::Path solution_path = prm.getPath();

    ros::Publisher inflated_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("inflated_map", 1);
    nav_msgs::OccupancyGrid::Ptr inflated_map = prm.getInflatedMap();

    nav_msgs::OccupancyGrid publishable_map;
    publishable_map.data = inflated_map->data;
    publishable_map.header = inflated_map->header;
    publishable_map.info = inflated_map->info;

    while (ros::ok()) {
        solution_path_publisher.publish(solution_path);
        inflated_map_publisher.publish(publishable_map);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    return 0;
}