#!/usr/bin/env python3

# Copyright (c) 2022 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import glob
import os
import time

from ament_index_python.packages import get_package_share_directory
import termcolor

# TIMEOUT is same as Autoware Evaluation Dashboard's timeout
TIMEOUT = 900


def count_engine_files() -> int:
    engine_file_regex = os.path.join(
        get_package_share_directory("lidar_centerpoint"), "data", "*.engine"
    )
    engine_files = glob.glob(engine_file_regex)
    return len(engine_files)


def is_engine_files_generated() -> bool:
    if count_engine_files() < 2:
        return False
    return True


def main():
    counter = 0
    while (counter < TIMEOUT) and not is_engine_files_generated():
        termcolor.cprint("Waiting for lidar_centerpoint engine files to be generated...", "yellow")
        counter += 1
        time.sleep(1.0)
    termcolor.cprint(
        "lidar_centerpoint engine files are generated.",
        "blue",
        "on_grey",
        attrs=["bold"],
    )
    # ファイルがある場合は、即座に実行されてしまうので、10秒待つ
    time.sleep(10.0)


if __name__ == "__main__":
    main()
