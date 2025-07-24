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

from abc import ABC, abstractmethod


class GoalGenerator(ABC):
    """
    Parent class for the Goal generators
    """

    def __init__(self):
        pass

    @abstractmethod
    def generate_goal(self, max_num_of_trials=2000):
        """
        Generate the goal.

        Parameters
        ----------
        max_num_of_trials: maximum number of pose generations when generated pose keep is not a valid pose.

        Returns
        -------
        [List][Pose]: Pose in format [pose.x,pose.y,orientaion.x,orientaion.y,orientaion.z,orientaion.w]
        """
        pass
