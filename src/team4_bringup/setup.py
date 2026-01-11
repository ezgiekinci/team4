import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'team4_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(), # find_packages(where='.') yerine bunu deneyin
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diakamos',
    maintainer_email='diakamos@todo.todo',
    description='Team 4 project: Multi-floor navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'floor_id_from_apriltag = team4_bringup.floor_id_from_apriltag:main',
            'map_switcher = team4_bringup.map_switcher:main',
        ],
    },
)
