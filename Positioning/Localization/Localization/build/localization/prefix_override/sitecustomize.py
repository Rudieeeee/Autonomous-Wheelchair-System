import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/engineer/ros2_ws/src/Autonomous-Wheelchair-System/Positioning/Localization/Localization/install/localization'
