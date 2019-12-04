#!/bin/bash

set -e

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
rosrun object_detection ros_node.py &
rosrun object_detection _inference.py
