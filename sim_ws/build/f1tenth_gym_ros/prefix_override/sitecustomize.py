import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vboxuser/sim_ws/install/f1tenth_gym_ros'
