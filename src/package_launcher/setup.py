from setuptools import find_packages, setup

package_name = 'package_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/launch_packages.launch.py']),
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
        ],
    },
)
