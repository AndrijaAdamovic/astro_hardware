from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'astro_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrija',
    maintainer_email='andrija.adamov@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astro_serial_node = astro_hardware.astro_serial_node:main',
            'reset_odom_service = astro_hardware.astro_serial_node:main',

        ],
    },
)
