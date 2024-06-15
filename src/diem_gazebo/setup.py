import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'diem_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        #Launch
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        #Worlds
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'worlds', 'diem_map'), glob(os.path.join('worlds', 'diem_map', '*.*'))),
        (os.path.join('share', package_name, 'worlds', 'diem_map', 'meshes'), glob(os.path.join('worlds', 'diem_map', 'meshes', '*.stl'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gdesimone',
    maintainer_email='44608428+gdesimone97@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
