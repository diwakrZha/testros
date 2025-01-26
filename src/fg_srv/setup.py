# /setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'fg_srv'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'rest_api'],
    package_dir={
        package_name: 'fg_srv',
        'rest_api': 'rest_api'
    },
    data_files=[
        # Install marker file in package index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        # Include package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Diwaker',
    maintainer_email='diwaker@dj.dj',
    description='FleetG Mission Management System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_api = rest_api.app:main',
            'mission_retrieval_node = fg_srv.rn1:main',
            'mission_subscriber_node = fg_srv.rn2:main',
            'mission_robot_controller = fg_srv.mission_robot_controller:main',
        ],
    },
)