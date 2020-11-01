import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pid_controller
from pid_controller.paths import PROJECT_ROOT_PATH

NOTEBOOKS_TEST_ASSETS_PATH = PROJECT_ROOT_PATH / 'notebooks' / 'assets'

