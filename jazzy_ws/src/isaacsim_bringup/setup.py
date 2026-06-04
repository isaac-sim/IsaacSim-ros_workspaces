from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isaacsim_bringup'

setup(
    name=package_name,
    version='0.2.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), ['scripts/open_isaacsim_stage.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Sim',
    maintainer_email='isaac-sim-maintainers@nvidia.com',
    description='The isaacsim_bringup package that contains the script which can be used to launch Isaac Sim as a ROS2 node from a launch file.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'run_isaacsim = isaacsim_bringup.run_isaacsim:main',
        ],
    },
)
