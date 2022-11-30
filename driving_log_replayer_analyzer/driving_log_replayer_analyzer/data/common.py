from dataclasses import dataclass
from typing import Dict


@dataclass
class Common:
    timestamp_system: float = -1.0
    timestamp_ros: float = -1.0

    def __init__(self, json_dict: Dict) -> None:
        try:
            self.timestamp_system = json_dict["Stamp"]["System"]
            self.timestamp_ros = json_dict["Stamp"]["ROS"]
        except (KeyError, IndexError):
            pass

    def validate(self):
        return self.timestamp_system > 0.0 and self.timestamp_ros > 0.0
