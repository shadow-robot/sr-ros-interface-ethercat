# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from __future__ import absolute_import
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sr_robot_lib'],
    package_dir={'': 'python_lib'})

setup(**setup_args)
