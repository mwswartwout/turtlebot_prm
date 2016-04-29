//
// Created by matt on 4/29/16.
// Adapted from the OMPL demo file RigidBodyPlanning.cpp
//

#include <ros/ros.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
//#include <ompl/config.h>
//#include <iostream>

bool isStateValid(const ompl::base::State *state) {
    // First cast abstract state into expected SE2 state
    const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();

    // Get first component of state and cast into real vector state space
    const ompl::base::RealVectorStateSpace::StateType *pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    // Get second component of state and cast into SO(2) space
    const ompl::base::SO2StateSpace::StateType *rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1);

    // TODO add validity checking here for state position and rotation
    // All rotations will be valid
    // Need to get occupancy map from ros message and then check position here
    bool valid_state = false;

    return valid_state;
}

void plan() {
    // Construct the space we are planning in
    ompl::base::StateSpacePtr space(new ompl::base::SE2StateSpace());

    // Set bounds for the R^2 part of SE(2)
    ompl::base::RealVectorBounds bounds(2);
    // TODO set theses bounds based on the occupancy map
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // Define a simple setup class
    ompl::geometric::SimpleSetup simple_setup(space);

    // Set state validity checking for this space
    simple_setup.setStateValidityChecker(boost::bind(&isStateValid, _1));

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

int main() {
    ROS_INFO("Planning for motion with OMPL");
    plan();
    return 0;
}