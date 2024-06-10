from setuptools import find_packages, setup
import os
import glob

package_name = 'sig_rec'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'config'), glob('config/*.h5'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apagano',
    maintainer_email='44608428+gdesimone97@users.noreply.github.com',
    description='Package for obtaining data from camera.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "decode_mod = sig_rec.signal_recognize:main",
            "trafficboi = sig_rec.traffic_recognize:main"
        ],
    },
)
