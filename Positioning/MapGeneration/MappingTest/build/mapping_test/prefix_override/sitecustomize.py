import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rudrh/Autonomous-Wheelchair-System/Positioning/MapGeneration/MappingTest/install/mapping_test'
