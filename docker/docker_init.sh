#!/bin/sh/
source ~/basilisk/.venv/bin/activate
source ~/example_ws/install/setup.bash
pip install -r ~/example_ws/src/ros2basilisk/requirements.txt


# ros2 launch ros2basilisk lqr_launch.py ws:="/home/ubuntu/example_ws/"