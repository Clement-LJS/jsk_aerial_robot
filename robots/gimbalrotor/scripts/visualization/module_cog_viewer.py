#!/usr/bin/env python3
"""Standalone interactive CoG inspector; no robot commands or TF broadcasts."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cog_tools.runtime import main

if __name__ == '__main__':
    main(gui=True)
