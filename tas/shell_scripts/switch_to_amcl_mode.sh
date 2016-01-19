#! /bin/bash
# Executed on button press, when first round is complete and mode should be switched to second round mode

# Save hector map to file
rosrun map_server map_saver -f "$(rospack find tas)/launch/config/map_server/hectormap"

# kill ROS nodes we don't need anymore
kill $(pidof hector_mapping) $(pidof ekf_localization_node)

# ROS-Launch the nodes needed for amcl mode
roslaunch tas odom_amcl.launch
