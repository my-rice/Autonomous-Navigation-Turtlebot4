from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'config_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.yaml'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.pgm'))),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giovanni',
    maintainer_email='g.intonti@studenti.unisa.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nessun_nodo = config_pkg.nessun_nodo:main'
        ],
    },
)
