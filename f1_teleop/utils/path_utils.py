import os

PACKAGE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # .../f1_teleop
PROJECT_ROOT = os.path.dirname(PACKAGE_PATH)                                  # .../f1_tianji_teleop
ASSET_PATH = os.path.join(PROJECT_ROOT, "assets")
DEFAULT_CONFIG_PATH = os.path.join(PACKAGE_PATH, "config", "f1_teleop.yaml")
