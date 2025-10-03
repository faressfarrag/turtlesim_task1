import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/faress-farrag/ros2_ws/install/my_turtle_pkg'
