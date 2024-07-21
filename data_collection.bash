#!/bin/bash

positions=(0.0 0.01 0.02 0.03 0.04)

for i in "${positions[@]}"; do

  echo "Moving to position: $i"

  rosrun cwru_dvrk_control go_ecm 0 -1 $i 0 2

  rosrun dvrk_structure_light save_images.py

  rostopic echo /ECM/joint_states/position -n1

done
