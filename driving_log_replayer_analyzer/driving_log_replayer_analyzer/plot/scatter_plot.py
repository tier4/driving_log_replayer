from pathlib import Path

import plotly.express as px

from .plot_base import PlotBase


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
        self._fig.update_traces(marker=dict(size=4), selector=dict(mode="markers"))

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
        self._fig.update_traces(marker=dict(size=4), selector=dict(mode="markers"))

    def use_boolean_tick(self):
        self._fig.update_yaxes(range=[-0.066, 1.066])
        self._fig.update_layout(
            yaxis=dict(tickmode="array", tickvals=[0, 1], ticktext=["False", "True"])
        )

    def add_vert_line(self, vert_line_config: dict, output_dir: Path):
        for key, val in vert_line_config.items():
            if key in str(output_dir):
                self._fig.add_vline(x=val, line_width=3, line_dash="dash", line_color="green")
