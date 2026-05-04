import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monti/robot_localization_ws/ros2_ws/install/control_nodes'
