#!/bin/bash
sshpass -p "anveshak" ssh -o StrictHostKeyChecking=no anveshak@192.168.0.10 'bash -c "source /opt/ros/melodic/setup.bash && export ROS_IP=192.168.0.10 && cd caesar2020/ && . ./devel/setup.bash && rosrun arm run.py"'
