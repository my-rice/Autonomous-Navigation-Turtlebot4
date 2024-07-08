from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'discovery_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='giovanni',
    maintainer_email='g.intonti@studenti.unisa.it',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'discovery = discovery_pkg.discovery:main'
        ],
    },
)
