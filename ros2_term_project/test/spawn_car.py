import os
import sys


os.system("ros2 run gazebo_ros spawn_entity.py -database prius_hybrid -entity PR001 -x 93 -y -11.7 -Y -1.57 "
          "-robot_namespace pr001 ")
os.system(" ros2 run gazebo_ros spawn_entity.py -database prius_hybrid -entity PR002 -x 93 -y -15.7 -Y -1.57 "
          "-robot_namespace pr002 ")
