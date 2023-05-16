#!/usr/bin/python3
"""
Setup script for Thermostat EEM
"""
from setuptools import setup, find_packages

setup(
    name="thermostat",
    packages=find_packages(),
    # Keep versions in Cargo.toml and py/setup.py synchronized.
    version="0.1.0",
    description="Thermostat Utilities",
    author="QUARTIQ GmbH",
    license="MIT",
    install_requires=[
        "stabilizer@git+https://github.com/quartiq/stabilizer@f4055aa#subdirectory=py",
        "miniconf-mqtt@git+https://github.com/quartiq/miniconf@8be449d#subdirectory=py/miniconf-mqtt",
    ],
)
