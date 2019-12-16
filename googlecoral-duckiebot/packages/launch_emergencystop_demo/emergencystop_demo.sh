#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
rosrun object_detection ros_node_emergencystop_demo.py &
rosrun object_detection _inference.py
