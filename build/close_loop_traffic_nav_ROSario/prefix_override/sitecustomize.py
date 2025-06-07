import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xyg/ros2_ws/src/install/close_loop_traffic_nav_ROSario'
