#!/usr/bin/env python3

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


setup_args = generate_distutils_setup(
    packages=["rm4d_sim_integration"],
    package_dir={"": "src"},
)

setup(**setup_args)
