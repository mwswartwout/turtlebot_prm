#TurtleBot Probabilistic Roadmap Implementation

##Simulation launch files
To run gmapping and create a map:
1. `roslaunch turtlebot_prm sim_with_mapping.launch`
2. `roslaunch turtlebot_teleop keyboard_teleop.launch`
3. Drive TurtleBot around in teleop mode, and then run `rosrun map_server map_saver -f /path/to/map_file`

To run amcl localization against known map:
1. `roslaunch turtlebot_prm sim_with_amcl.launch`
2. `roslaunch turtlebot_teleop keyboard_teleop.launch`, if desired