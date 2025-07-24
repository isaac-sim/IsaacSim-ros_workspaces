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

from setuptools import setup
from glob import glob
import os

package_name = "isaac_ros_navigation_goal"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, package_name + "/goal_generators"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name + "/assets", glob("assets/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="isaac sim",
    maintainer_email="isaac-sim@todo.todo",
    description="Package to set goals for navigation stack.",
    license="NVIDIA Isaac ROS Software License",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["SetNavigationGoal = isaac_ros_navigation_goal.set_goal:main"]},
)
