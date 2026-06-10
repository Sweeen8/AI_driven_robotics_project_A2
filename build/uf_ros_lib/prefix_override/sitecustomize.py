import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/Git-projects/AI_driven_robotics_project_A2/install/uf_ros_lib'
