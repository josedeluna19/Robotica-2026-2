from setuptools import find_packages
from setuptools import setup

setup(
    name='click_to_motion_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('click_to_motion_msgs', 'click_to_motion_msgs.*')),
)
