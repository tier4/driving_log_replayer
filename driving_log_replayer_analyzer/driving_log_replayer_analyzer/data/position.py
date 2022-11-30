from dataclasses import dataclass
import math
from typing import Dict


@dataclass
class Position:
    x: float = None  # vehicle front
    y: float = None  # vehicle side (right side is positive)
    z: float = None

    def __init__(self, data: Dict = {}) -> None:
        self.try_parse_dict(data)

    def try_parse_dict(self, data):
        if isinstance(data, dict):
            try:
                self.y = data["x"]
                self.x = data["y"]
                self.z = data["z"]
            except KeyError:
                pass
        elif isinstance(data, list):
            try:
                self.y = data[0]
                self.x = data[1]
                self.z = data[2]
            except IndexError:
                pass
        else:
            raise NotImplementedError("Input data should be a dict or list.")

    def validate(self):
        return self.x is not None and self.y is not None and self.z is not None

    def get_xy_distance(self) -> float:
        if self.validate():
            return math.hypot(self.x, self.y)
        else:
            return None

    def add_overhang(self, val: float):
        self.y = self.y + val

    def sub_overhang(self, val: float):
        self.y = self.y - val
