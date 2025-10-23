from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cam_director'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
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
            'cam_director_node = cam_director.director:main',
        ],
    },
)
