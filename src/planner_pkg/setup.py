from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'planner_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include the configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include the rviz configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davide',
    maintainer_email='dvdrsi2017@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = planner_pkg.planner:main'
        ],
    },
)
