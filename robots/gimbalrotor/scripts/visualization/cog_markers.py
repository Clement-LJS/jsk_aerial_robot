#!/usr/bin/env python3
"""Headless preview/live comparison publisher. Qt is never imported."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from cog_tools.runtime import main

if __name__ == '__main__':
    main()
