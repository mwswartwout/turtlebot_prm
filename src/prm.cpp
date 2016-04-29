//
// Created by matt on 4/29/16.
// Adapted from the OMPL demo file RigidBodyPlanning.cpp
//

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

PRM::PRM(ros::NodeHandle& nodeHandle, bool unknown = false, int threshold = 50) :
    nh(nodeHandle) {
    map_received = false;
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
}

void PRM::mapCallback( nav_msgs::OccupancyGrid new_map) {
    // Check to see if we've previously received a map
    if (map_received) {
        ROS_WARN("New map received, this shouldn't happen");
    }

    map = new_map; // Save new map that we've got
    map_received = true; // Mark that we've received a map

    // Figure out the x and y limits of our map
    geometry_msgs::Polygon bounds = occupancy_grid_utils::gridPolygon(map.info);
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

    if (occupancy_grid_utils::withinBounds(map.info, point)) { // First check if we're in bounds
        ROS_DEBUG("State is in bounds... now checking occupancy...");
        int index = occupancy_grid_utils::pointIndex(map.info, point);
        int occupied_status = map.data[index];

        if (unknown_okay && occupied_status == -1) { // Are unkown cells valid states?
            ROS_DEBUG("State occupancy is unknown and unknown_okay has been set to true");
            return true;
        }
        else if (map.data[index] < occupied_threshold) {
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

void PRM::plan() {
    // Construct the space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

    ROS_INFO("Waiting for a map...");
    while (!map_received) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO("Got a map!");

    // Set bounds for the R^2 part of SE(2)
    ompl::base::RealVectorBounds bounds(2);
    // TODO set theses bounds based on the occupancy map
    bounds.setLow(0, min_x);
    bounds.setHigh(0, max_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(1, max_y);
    ROS_INFO_STREAM("Set X bounds of map to " << min_x << " -> " << max_x);
    ROS_INFO_STREAM("Set Y bounds of map to " << min_y << " -> " << max_y);

    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // Define a simple setup class
    ompl::geometric::SimpleSetup simple_setup(space);

    // Set state validity checking for this space
    simple_setup.setStateValidityChecker(boost::bind(&PRM::isStateValid, this, _1));

    // Create start state
    ompl::base::ScopedState<> start(space);
    start.random(); // TODO this should be taken from amcl/odom, not random

    // Create goal state
    ompl::base::ScopedState<> goal(space);
    goal.random(); // TODO this should be taken interactively from the user

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
        simple_setup.getSolutionPath().print(std::cout); // TODO see if this can be switched to ROS_INFO
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
    return 0;
}