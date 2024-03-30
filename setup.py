#!/usr/bin/env python3

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=[
        "src/ball_detector.py",
        "src/usb_camera.py",
    ],
    package_dir={"": "src"},
)

setup(**d)
