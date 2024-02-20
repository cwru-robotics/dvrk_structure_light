#!/bin/bash

positions=(0.0 0.006 0.012 0.018 0.024 0.030 0.036 0.042 0.048)

for i in "${positions[@]}"; do

  echo "Moving to position: $i"

  rosrun cwru_dvrk_control go_ecm 1.527652 -0.295480 $i -0.606090 1

  rosrun hw_camera save_images.py

  rostopic echo /ECM/joint_states/position -n1

done