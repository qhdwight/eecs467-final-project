#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=[
        "src/ball_detector.py",
        "src/bot_detector.py",
        "src/simulator.py",
        "src/diff_drive.py",
    ],
    package_dir={"": "src"},
)

setup(**d)
