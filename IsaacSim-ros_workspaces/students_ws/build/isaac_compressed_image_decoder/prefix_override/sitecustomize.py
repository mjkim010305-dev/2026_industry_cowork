import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ciderlab-server3/workspace/2026_industry_cowork/IsaacSim-ros_workspaces/humble_ws/install/isaac_compressed_image_decoder'
