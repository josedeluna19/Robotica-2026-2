from setuptools import setup
import os
from glob import glob

package_name = 'click_to_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Click to motion package - planar + spatial',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'urdf_publisher = click_to_motion.urdf_publisher:main',
            'click_to_motion_planar = click_to_motion.planar_click_to_motion:main',
            'click_to_motion_spatial = click_to_motion.spatial_click_to_motion:main',
        ],
    },
)
