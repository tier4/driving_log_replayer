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

import plotly.express as px

from .plot_base import PlotBase


class LinePlot(PlotBase):
    def plot(self, title="", xlabel="", ylabel="", use_boolean_tick=False):
        self._fig = px.line(
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

        if use_boolean_tick:
            self._fig.update_layout(
                yaxis=dict(  # noqa: C408
                    tickmode="array", tickvals=[0, 1], ticktext=["False", "True"]
                )
            )
