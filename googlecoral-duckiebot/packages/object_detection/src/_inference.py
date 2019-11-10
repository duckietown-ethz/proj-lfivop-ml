#!/usr/bin/env python
import subprocess

subprocess.check_call('python3 /code/catkin_ws/src/coral/packages/object_detection/src/inference.py', shell=True)