import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/julio/Repo_Robotica_2026_2/Robotica-2026-2/install/rrr_control'
