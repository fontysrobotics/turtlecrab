from setuptools import setup
from glob import glob
import os

package_name = 'turtlecrab'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/nav'), glob('config/nav/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marian',
    maintainer_email='marian@noxrobotics.nl',
    description='Mini Holonomic robotic',
    license='bsd-3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holo_driver = turtlecrab.holo_driver:main'
        ],
    },
)
