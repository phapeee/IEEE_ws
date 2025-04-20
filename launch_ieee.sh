#!/bin/bash
source ~/.bashrc
source /opt/ros/noetic/setup.bash
source ~/IEEE_ws/devel_isolated/setup.bash

roslaunch ieee_master ieee_master.launch
