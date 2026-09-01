#!/usr/bin/env python3

from setuptools import find_packages, setup


setup(
    name="ground_manipulator_runtime",
    version="1.0.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
)
