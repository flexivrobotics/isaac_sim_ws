# Copyright (c) 2026, Flexiv Ltd. All rights reserved.
#
# Lula motion generation config of the Flexiv Enlight L, shipped in this repo's
# assets/enlight_l/rmpflow, for the RMPflow controller and the kinematics solver.

import os

# This file is <repo>/flexiv_isaac/motion_config.py
_REPO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
_CONFIG_DIR = os.path.join(_REPO_DIR, "assets", "enlight_l", "rmpflow")

URDF_PATH = os.path.join(_CONFIG_DIR, "enlight_l.urdf")
ROBOT_DESCRIPTION_PATH = os.path.join(_CONFIG_DIR, "enlight_l_robot_description.yaml")
RMPFLOW_CONFIG_PATH = os.path.join(_CONFIG_DIR, "enlight_l_rmpflow_config.yaml")
END_EFFECTOR_FRAME = "flange"
