import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ryder/Documents/as4/assignment4-ros2/install/tb3_autonomous'
