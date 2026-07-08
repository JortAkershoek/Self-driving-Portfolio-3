#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Make catkin packages discoverable by roslaunch / rospack.
# template-basic does not run catkin_make, so we register them manually.
export ROS_PACKAGE_PATH="${DT_REPO_PATH}/packages:${ROS_PACKAGE_PATH}"

# Set vehicle name fallback for DTS usage.
# Prefer VEHICLE_NAME, then ROBOT_NAME, then HOSTNAME.
export VEHICLE_NAME=${VEHICLE_NAME:-${ROBOT_NAME:-${HOSTNAME:-duckiebot}}}
# Prefer an explicitly configured maze file, otherwise use the bundled default.
export DUCKIE_MAZE_MAP=${DUCKIE_MAZE_MAP:-${DT_REPO_PATH}/assets/maze_map.yaml}

# Launch the full maze navigation stack
dt-exec roslaunch maze_path_planner maze_navigation.launch \
  veh:=${VEHICLE_NAME} \
  start_node:=${START_NODE:-A} \
  goal_node:=${GOAL_NODE:-E}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
