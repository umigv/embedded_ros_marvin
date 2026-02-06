import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/complete_embedded_ws/embedded_ros_marvin/install/embedded_ros_marvin'
