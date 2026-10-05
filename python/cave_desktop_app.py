#!/usr/bin/env python3
"""
Forwarder to root cave_desktop_app.py or standalone launcher.
"""
import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from cave_desktop_app import main

if __name__ == '__main__':
    main()
