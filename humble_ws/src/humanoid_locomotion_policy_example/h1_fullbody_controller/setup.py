# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'h1_fullbody_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'policy'),
            glob('policy/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Isaac Sim',
    maintainer_email='isaac-sim-maintainers@nvidia.com',
    description='Fullbody controller for the H1 humanoid robot using a neural network policy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'h1_fullbody_controller = h1_fullbody_controller.h1_fullbody_controller:main',
        ],
    },
)
