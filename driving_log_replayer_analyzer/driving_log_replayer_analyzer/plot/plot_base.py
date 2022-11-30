from pathlib import Path
from typing import Dict
from typing import List

import pandas as pd


class PlotBase:
    def __init__(self) -> None:
        self._df = pd.DataFrame(columns=["x", "y", "color", "legend"])
        self._fig = None

    def add_data(self, data: List, legend: str = ""):
        for item in data:
            item.append(legend)
        df = pd.DataFrame(data, columns=["x", "y", "color", "legend"])
        self._df = pd.concat([self._df, df])

    def add_data_with_hover(self, data: List[Dict], legend: str = ""):
        for item in data:
            item["legend"] = legend
        df = pd.DataFrame.from_dict(data)
        self._df = pd.concat([self._df, df])

    def _get_min(self, column: str):
        return self._df[column].min()

    def _get_max(self, column: str):
        return self._df[column].max()

    def _get_range(self, column: str):
        return self._get_max(column) - self._get_min(column)

    def _get_abs_max(self, column: str):
        return self._df[column].abs().max()

    def save_plot(self, path: Path):
        self._save(self._fig, path)

    def _save(self, fig, path: Path):
        fig.write_image(path.with_suffix(".png"))
        fig.write_html(path.with_suffix(".html"))

    def _get_hover_column(self) -> List:
        hover_list = self._df.columns.tolist()
        hover_list.remove("x")
        hover_list.remove("y")
        hover_list.remove("color")
        return hover_list

    def set_tick_span(self, x: float = None, y: float = None):
        if x is not None:
            self._fig.update_xaxes(dtick=x)
        if y is not None:
            self._fig.update_yaxes(dtick=y)

    def set_xy_range(self, xy_range: dict):
        if "x" in xy_range.keys():
            self._fig.update_xaxes(range=[xy_range["x"][0], xy_range["x"][1]])
        if "y" in xy_range.keys():
            self._fig.update_yaxes(range=[xy_range["y"][0], xy_range["y"][1]])
