import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ciderlab-server3/adaptive_manip_ws/test/install/perception_pkg'
