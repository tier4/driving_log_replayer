import plotly.express as px

from .plot_base import PlotBase


class BirdViewPlot(PlotBase):
    FIGURE_SIZE = 800
    MARGIN = 3.0

    def set_scale(self, scale, origin: bool = True):
        if scale == "auto":
            if origin:
                # 原点が中央になるように調整
                max_abs_x = self._get_abs_max("x") + self.MARGIN
                max_abs_y = self._get_abs_max("y") + self.MARGIN
                self._fig.update_xaxes(range=[-max_abs_x, max_abs_x])
                self._fig.update_yaxes(range=[-max_abs_y, max_abs_y])
            else:
                # Default config
                pass
        elif scale == "square":
            if origin:
                max_abs_x = self._get_abs_max("x") + self.MARGIN
                max_abs_y = self._get_abs_max("y") + self.MARGIN
                max_abs = max(max_abs_x, max_abs_y)
                self._fig.update_xaxes(range=[-max_abs, max_abs])
                self._fig.update_yaxes(range=[-max_abs, max_abs])
            else:
                if self._get_range("x") > self._get_range("y"):
                    # Use x range
                    plot_range = self._get_range("x") + self.MARGIN
                else:
                    # Use y range
                    plot_range = self._get_range("y") + self.MARGIN
                self._fig.update_xaxes(
                    range=[
                        self._get_min("x") - self.MARGIN,
                        self._get_min("x") + plot_range,
                    ]
                )
                self._fig.update_yaxes(
                    range=[
                        self._get_min("y") - self.MARGIN,
                        self._get_min("y") + plot_range,
                    ]
                )
        else:
            # specify axis
            if origin:
                self._fig.update_xaxes(range=[-scale, scale])
                self._fig.update_yaxes(range=[-scale, scale])
            else:
                pass

        # 座標軸を表示
        self._fig.update_xaxes(zeroline=True, zerolinewidth=2, zerolinecolor="LightPink")
        self._fig.update_yaxes(zeroline=True, zerolinewidth=2, zerolinecolor="LightPink")

    def plot(
        self,
        title: str = "",
        xlabel: str = "",
        ylabel: str = "",
    ):
        """Plotを作成して保存する

        Args:
            path (Path): 保存先。拡張子はpng
            title (str, optional): タイトル. Defaults to "".
            xlabel (str, optional): x軸のラベル. Defaults to "".
            ylabel (str, optional): y軸のラベル. Defaults to "".
            origin (bool, optional): 原点中心にする場合はTrue. Defaults to True.
        """
        self._fig = px.scatter(
            self._df,
            x="x",
            y="y",
            color="color",
            color_discrete_map={
                "Success": "green",
                "Warn": "yellow",
                "Fail": "red",
            },
            width=self.FIGURE_SIZE,
            height=self.FIGURE_SIZE,
        )

        self._fig.update_traces(marker=dict(size=2), selector=dict(mode="markers"))

        self._fig.update_layout(
            title=title,
            xaxis_title=xlabel,
            yaxis_title=ylabel,
        )

    def plot_with_hover(
        self,
        title: str = "",
        xlabel: str = "",
        ylabel: str = "",
        origin: bool = True,
    ):
        """Plotを作成して保存する

        Args:
            path (Path): 保存先。拡張子はpng
            title (str, optional): タイトル. Defaults to "".
            xlabel (str, optional): x軸のラベル. Defaults to "".
            ylabel (str, optional): y軸のラベル. Defaults to "".
            origin (bool, optional): 原点中心にする場合はTrue. Defaults to True.
        """
        # hover dataをdfから抽出
        hover_list = self._get_hover_column()

        self._fig = px.scatter(
            self._df,
            x="x",
            y="y",
            color="color",
            color_discrete_map={
                "Success": "green",
                "Warn": "yellow",
                "Fail": "red",
            },
            hover_data=hover_list,
            width=self.FIGURE_SIZE,
            height=self.FIGURE_SIZE,
        )

        self._fig.update_traces(marker=dict(size=2), selector=dict(mode="markers"))

        self._fig.update_layout(
            title=title,
            xaxis_title=xlabel,
            yaxis_title=ylabel,
        )
