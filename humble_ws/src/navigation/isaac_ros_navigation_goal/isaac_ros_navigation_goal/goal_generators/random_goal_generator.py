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

import numpy as np
from .goal_generator import GoalGenerator


class RandomGoalGenerator(GoalGenerator):
    """
    Random goal generator.

    parameters
    ----------
    grid_map: GridMap Object
    distance: distance in meters to check vicinity for obstacles.
    """

    def __init__(self, grid_map, distance):
        self.__grid_map = grid_map
        self.__distance = distance

    def generate_goal(self, max_num_of_trials=1000):
        """
        Generate the goal.

        Parameters
        ----------
        max_num_of_trials: maximum number of pose generations when generated pose keep is not a valid pose.

        Returns
        -------
        [List][Pose]: Pose in format [pose.x,pose.y,orientaion.x,orientaion.y,orientaion.z,orientaion.w]
        """
        range_ = self.__grid_map.get_range()
        trial_count = 0
        while trial_count < max_num_of_trials:
            x = np.random.uniform(range_[0][0], range_[0][1])
            y = np.random.uniform(range_[1][0], range_[1][1])
            orient_x = np.random.uniform(0, 1)
            orient_y = np.random.uniform(0, 1)
            orient_z = np.random.uniform(0, 1)
            orient_w = np.random.uniform(0, 1)
            if self.__grid_map.is_valid_pose([x, y], self.__distance):
                goal = [x, y, orient_x, orient_y, orient_z, orient_w]
                return goal
            trial_count += 1
