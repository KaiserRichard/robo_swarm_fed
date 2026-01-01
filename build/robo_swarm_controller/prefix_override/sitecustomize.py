import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/richard/ros2_ws/src/robo_swarm_fed/install/robo_swarm_controller'
