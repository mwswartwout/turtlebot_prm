//
// Created by matt on 4/29/16.
// Adapted from the OMPL demo file RigidBodyPlanning.cpp
// BaseGlobalPlanner modifications adapted from carrot_planner
//

// TODO see about expanding this w/ controls
// TODO turn this into a plugin that extends the nav_core::BaseGlobalPlanner so that it can be dropped into the navstack
// TODO make this plan for optimal path length

#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <turtlebot_prm/prm.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(PRM, nav_core::BaseGlobalPlanner)

geometry_msgs::Quaternion convertPlanarPhiToQuaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

PRM::PRM() :
        costmap_ros(NULL),
        map_received(false),
        pose_received(false),
        initialized(false),
        robot_radius(0.2),
        unknown_okay(false),
        occupied_threshold(50),
        max_x(std::numeric_limits<int>::min()),
        max_y(std::numeric_limits<int>::min()),
        min_x(std::numeric_limits<int>::max()),
        min_y(std::numeric_limits<int>::max()) {
    ROS_INFO("In constructor of PRM");
    initializeSubscribers();
}

PRM::PRM(std::string name, costmap_2d::Costmap2DROS* new_costmap_ros) :
        costmap_ros(NULL),
        map_received(false),
        pose_received(false),
        initialized(false),
        unknown_okay(false),
        occupied_threshold(50),
        max_x(std::numeric_limits<int>::min()),
        max_y(std::numeric_limits<int>::min()),
        min_x(std::numeric_limits<int>::max()),
        min_y(std::numeric_limits<int>::max()) {
    ROS_INFO("In constructor of PRM");
    initializeSubscribers();
    initialize(name, new_costmap_ros);
}

void PRM::initialize(std::string name, costmap_2d::Costmap2DROS* new_costmap_ros) {
    if (!initialized) {
        costmap_ros = new_costmap_ros;
        costmap = costmap_ros->getCostmap();
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size, costmap->getResolution());
        private_nh.param("min_dist_from_robot", robot_radius, 0.2);
        world_model = new base_local_planner::CostmapModel(*costmap);

        plan_publisher = private_nh.advertise<nav_msgs::Path>("prm_plan", 1, true);
        initialized = true;
    }
    else {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
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

    geometry_msgs::Point point;
    point.x = se2state->getX();
    point.y = se2state->getY();
    ROS_DEBUG_STREAM("Checking validity of state ( " << point.x << ", " << point.y <<")");

    std::vector<geometry_msgs::Point> footprint;
    footprint.push_back(point);

    double point_cost = world_model->footprintCost(point, footprint, robot_radius, robot_radius);

    if (point_cost >= 0) { // No collision detected
        return true;
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
        double yaw = ompl_states[i]->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
        pose.pose.orientation = convertPlanarPhiToQuaternion(yaw);
        ros_path.poses.push_back(pose);
    }

    return ros_path;
}

nav_msgs::OccupancyGrid::Ptr PRM::getInflatedMap(){
    return inflated_map;
}

bool PRM::makePlan(const geometry_msgs::PoseStamped& ros_start,
                   const geometry_msgs::PoseStamped& ros_goal,
                   std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized) {
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    // Construct the space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());
    ompl::base::SpaceInformationPtr space_information(new ompl::base::SpaceInformation(space));

    // Set bounds for the R^2 part of SE(2)
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, costmap->getOriginX());
    bounds.setHigh(0, costmap->getSizeInMetersX() - costmap->getOriginX());
    bounds.setLow(1, costmap->getOriginY());
    bounds.setHigh(1, costmap->getSizeInMetersY() - costmap->getOriginY());
    ROS_DEBUG_STREAM("Set X bounds of map to " << 0 << " -> " << costmap->getSizeInMetersX());
    ROS_DEBUG_STREAM("Set Y bounds of map to " << 0 << " -> " << costmap->getSizeInMetersY());

    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // Define a simple setup class
    ompl::geometric::SimpleSetup simple_setup(space_information);

    // Let's use the PRM* optimizing planner
    ompl::base::PlannerPtr planner(new ompl::geometric::PRMstar(space_information));
    simple_setup.setPlanner(planner);

    // Let's optimize for shortest path length
    ompl::base::OptimizationObjectivePtr objective(new ompl::base::PathLengthOptimizationObjective(space_information));
    simple_setup.setOptimizationObjective(objective);

    // Set state validity checking for this space
    simple_setup.setStateValidityChecker(boost::bind(&PRM::isStateValid, this, _1));

    // Create start state
    ompl::base::ScopedState<> ompl_start(space);
    ompl_start->as<ompl::base::SE2StateSpace::StateType>()->setX(ros_start.pose.position.x);
    ompl_start->as<ompl::base::SE2StateSpace::StateType>()->setY(ros_start.pose.position.y);
    //start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start.orientation.); Don't need this right now
    ROS_DEBUG_STREAM("Set start pose to ( " << ros_start.pose.position.x << ", " << ros_start.pose.position.y << " )");

    // Create goal state
    ompl::base::ScopedState<> ompl_goal(space);
    ompl_goal->as<ompl::base::SE2StateSpace::StateType>()->setX(ros_goal.pose.position.x);
    ompl_goal->as<ompl::base::SE2StateSpace::StateType>()->setY(ros_goal.pose.position.y);
    ROS_DEBUG_STREAM("Set goal pose to ( " << ros_goal.pose.position.x << ", " << ros_goal.pose.position.y << " )");

    // Set the start and goal state
    simple_setup.setStartAndGoalStates(ompl_start, ompl_goal);

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
        nav_msgs::Path ros_solution_path = omplPathToRosPath(ompl_solution_path);

        // Populate the std::vector from the nav_msgs::Path
        geometry_msgs::PoseStamped temp_pose;
        for (unsigned i = 0; i < ros_solution_path.poses.size(); i++) {
            ros::Time plan_time = ros::Time::now();

            temp_pose.header.stamp = plan_time;
            temp_pose.header.frame_id = costmap_ros->getGlobalFrameID();
            temp_pose.pose = ros_solution_path.poses[i].pose;

            plan.push_back(temp_pose);
        }

        plan_publisher.publish(ros_solution_path);
        ros::spinOnce();
    }
    else {
        ROS_WARN("No solution found");
    }

    return true;
}

