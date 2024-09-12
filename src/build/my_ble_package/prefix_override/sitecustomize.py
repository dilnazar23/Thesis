import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nazar/ros_workspaces/Thesis/src/install/my_ble_package'
