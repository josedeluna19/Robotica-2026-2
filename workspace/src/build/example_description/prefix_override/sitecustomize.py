import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/Semestre-2026-1/Workspaces/example_ws/src/install/example_description'
