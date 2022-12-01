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
                yaxis=dict(tickmode="array", tickvals=[0, 1], ticktext=["False", "True"])
            )
