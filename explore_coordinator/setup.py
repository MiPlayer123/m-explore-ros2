from setuptools import setup
import os
from glob import glob

package_name = 'explore_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Alvarez',
    maintainer_email='candres.alv@gmail.com',
    description='Multi-robot exploration coordinator using Hungarian algorithm',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coordinator = explore_coordinator.coordinator_node:main',
        ],
    },
)
