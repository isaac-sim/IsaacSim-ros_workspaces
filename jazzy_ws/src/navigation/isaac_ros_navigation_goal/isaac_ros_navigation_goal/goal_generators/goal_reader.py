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

from .goal_generator import GoalGenerator


class GoalReader(GoalGenerator):
    def __init__(self, file_path):
        self.__file_path = file_path
        self.__generator = self.__get_goal()

    def generate_goal(self, max_num_of_trials=1000):
        try:
            return next(self.__generator)
        except StopIteration:
            return

    def __get_goal(self):
        for row in open(self.__file_path, "r"):
            yield list(map(float, row.strip().split(" ")))
