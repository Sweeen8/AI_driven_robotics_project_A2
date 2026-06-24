#!/bin/bash

cd ~/Git-projects/AI_driven_robotics_project_A2

source .xarm_venv/bin/activate

python movement/guarded_fast_sdk_controller.py

bash ~/Git-projects/AI_driven_robotics_project_A2/movement/run_movement.sh
