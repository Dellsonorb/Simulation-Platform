"""Append Ubuntu's pure Python ROS dependencies after the RM4D environment."""

import sys


SYSTEM_DIST_PACKAGES = "/usr/lib/python3/dist-packages"

if SYSTEM_DIST_PACKAGES not in sys.path:
    sys.path.append(SYSTEM_DIST_PACKAGES)
