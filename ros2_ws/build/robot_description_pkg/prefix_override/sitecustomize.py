import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yousefabdelhady/ros2_neurofuzzy_ekf_localization/ros2_ws/install/robot_description_pkg'
