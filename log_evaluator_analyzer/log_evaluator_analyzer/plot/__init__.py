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

from pathlib import Path

import pandas as pd


class PlotBase:
    def __init__(self) -> None:
        self._df = pd.DataFrame(columns=["x", "y", "color", "legend"])
        self._fig = None

    def add_data(self, data: list, legend: str = "") -> None:
        for item in data:
            item.append(legend)
        df = pd.DataFrame(data, columns=["x", "y", "color", "legend"])
        self._df = pd.concat([self._df, df])

    def add_data_with_hover(self, data: list[dict], legend: str = "") -> None:
        for item in data:
            item["legend"] = legend
        df = pd.DataFrame.from_dict(data)
        self._df = pd.concat([self._df, df])

    def _get_min(self, column: str):  # noqa
        return self._df[column].min()

    def _get_max(self, column: str):  # noqa
        return self._df[column].max()

    def _get_range(self, column: str):  # noqa
        return self._get_max(column) - self._get_min(column)

    def _get_abs_max(self, column: str):  # noqa
        return self._df[column].abs().max()

    def save_plot(self, path: Path) -> None:
        self._save(self._fig, path)

    def _save(self, fig, path: Path) -> None:  # noqa
        fig.write_image(path.with_suffix(".png"))
        fig.write_html(path.with_suffix(".html"))

    def _get_hover_column(self) -> list:
        hover_list = self._df.columns.tolist()
        hover_list.remove("x")
        hover_list.remove("y")
        hover_list.remove("color")
        return hover_list

    def set_tick_span(self, x: float | None = None, y: float | None = None) -> None:
        if x is not None:
            self._fig.update_xaxes(dtick=x)
        if y is not None:
            self._fig.update_yaxes(dtick=y)

    def set_xy_range(self, xy_range: dict) -> None:
        if "x" in xy_range:
            self._fig.update_xaxes(range=[xy_range["x"][0], xy_range["x"][1]])
        if "y" in xy_range:
            self._fig.update_yaxes(range=[xy_range["y"][0], xy_range["y"][1]])

    def to_dict(self) -> dict:
        return self._df.to_dict()
