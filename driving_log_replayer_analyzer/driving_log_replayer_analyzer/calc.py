# Copyright (c) 2022 TierIV.inc
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

import sys
from typing import List

import pandas as pd


def fail_3_times_in_a_row(data: List) -> List:
    """対象点から近いほうの点3点から、連続して3点Failしている点をFailとする変換を行う.

    Args:
        data (list): [距離, 0 or 1, Success or Fail]

    Returns:
        list: Inputと同じ形式のlist。2項目目の0 or 1が変更される。
    """
    WINDOW = 3

    df = pd.DataFrame(data, columns=["Dist", "Val", "Result"])

    # 距離順にソート
    df.sort_values("Dist", ascending=True, inplace=True)
    df.reset_index(inplace=True, drop=True)
    df["Val"] = df["Val"].rolling(WINDOW, min_periods=1).max()

    return df.to_numpy().tolist()


def get_min_range(data: List) -> float:
    df = pd.DataFrame(data, columns=["Dist", "Val", "Result"])

    # Val == 0はFail, 最初にFailした距離を探索する
    minimum_fail_dist = df[df["Val"] == 0].min()["Dist"]
    if pd.isnull(minimum_fail_dist):
        minimum_fail_dist = sys.float_info.max

    # Passしたもののうち、最大の距離を計算する。ただし、一度でもFailするとダメなので、その条件も加える。
    return df[(df["Val"] == 1) & (df["Dist"] < minimum_fail_dist)].max()["Dist"]
