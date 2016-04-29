//
// Created by matt on 4/29/16.
//

#ifndef TURTLEBOT_PRM_PRM_H
#define TURTLEBOT_PRM_PRM_H

#include <ompl/base/State.h>
#include <nav_msgs/OccupancyGrid.h>

class PRM{
public:
    PRM(ros::NodeHandle& nodeHandle, bool unknown, int threshold);
    void plan();

private:
    ros::NodeHandle nh;
    nav_msgs::OccupancyGrid map;
    ros::Subscriber map_subscriber;
    bool map_received;
    bool unknown_okay;
    int occupied_threshold;

    void initializeSubscribers();
    void mapCallback(nav_msgs::OccupancyGrid new_map);
    bool isStateValid(const ompl::base::State *state);
};

#endif //TURTLEBOT_PRM_PRM_H
