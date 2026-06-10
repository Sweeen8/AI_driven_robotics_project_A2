cd ~/Git-projects/AI_driven_robotics_project_A2
of
cd ~/AI_driven_robotics_project_A2

colcon build \
  --symlink-install \
  --packages-select a2_workcell_description

--

source /opt/ros/jazzy/setup.bash
source ~/Git-projects/AI_driven_robotics_project_A2/install/setup.bash
of
source ~/AI_driven_robotics_project_A2/install/setup.bash

--

ros2 launch \
  a2_workcell_description \
  lite6_workcell_bringup.launch.py

--

 2e terminal

source /opt/ros/jazzy/setup.bash
source ~/Git-projects/AI_driven_robotics_project_A2/install/setup.bash

ros2 run tf2_ros tf2_echo environment_root link_base
