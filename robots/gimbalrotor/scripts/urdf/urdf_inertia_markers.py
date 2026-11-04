#!/usr/bin/env python3
"""Compatible declared-URDF, link-local markers on /urdf/inertia_markers.

Parameters ~robot_description_param, ~tf_prefix and ~only_links are preserved.
Use cog_markers.py for independent reference/candidate comparison and aggregates.
"""
from pathlib import Path
import sys

# Installed entry points share a directory with cog_tools; source execution uses
# the adjacent visualization folder. Catkin relays execute this source file too.
_here = Path(__file__).resolve().parent
sys.path.insert(0, str(_here))
sys.path.insert(0, str(_here.parent / 'visualization'))
from cog_tools.runtime import main

if __name__ == '__main__':
    main(legacy=True)
