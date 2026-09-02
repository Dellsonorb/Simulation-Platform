#!/usr/bin/env python3

from setuptools import find_packages, setup


setup(
    name="p450_flight_facade",
    version="1.0.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
)
