import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rudrh/Autonomous-Wheelchair-System/Navigation/Pathfinding/NavigationTest/install/navigation_test'
