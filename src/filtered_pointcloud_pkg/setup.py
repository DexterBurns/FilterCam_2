from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'filtered_pointcloud_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/threshold_launch.launch.py']),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dexter',
    maintainer_email='dexter_burns456@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'threshold_node = filtered_pointcloud_pkg.threshold_node:main',
        ],
    },
)
