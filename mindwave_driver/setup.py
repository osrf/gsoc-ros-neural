#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_disutils_setup(
    scripts=['scripts/mindwave_node.py'],
    packages=['mindwave_driver'],
    package_dir={'':'src'}
)

setup(**d)