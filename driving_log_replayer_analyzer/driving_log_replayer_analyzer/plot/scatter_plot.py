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

import plotly.express as px

from . import PlotBase


class ScatterPlot(PlotBase):
    def plot(self, title="", xlabel="", ylabel=""):
        self._fig = px.scatter(
            self._df,
            x="x",
            y="y",
            color="legend",
        )
        self._fig.update_layout(
            title=title,
            xaxis_title=xlabel,
            yaxis_title=ylabel,
        )
        self._fig.update_traces(marker=dict(size=4), selector=dict(mode="markers"))  # noqa: C408

    def plot_with_hover(self, title="", xlabel="", ylabel=""):
        # hover dataをdfから抽出
        hover_list = self._get_hover_column()

        self._fig = px.scatter(
            self._df,
            x="x",
            y="y",
            color="legend",
            hover_data=hover_list,
        )
        self._fig.update_layout(
            title=title,
            xaxis_title=xlabel,
            yaxis_title=ylabel,
        )
        self._fig.update_traces(marker=dict(size=4), selector=dict(mode="markers"))  # noqa: C408

    def use_boolean_tick(self):
        self._fig.update_yaxes(range=[-0.066, 1.066])
        self._fig.update_layout(
            yaxis=dict(tickmode="array", tickvals=[0, 1], ticktext=["False", "True"])  # noqa: C408
        )

    def add_vert_line(self, vert_line_config: dict, output_dir: Path):
        for key, val in vert_line_config.items():
            if key in str(output_dir):
                self._fig.add_vline(x=val, line_width=3, line_dash="dash", line_color="green")
