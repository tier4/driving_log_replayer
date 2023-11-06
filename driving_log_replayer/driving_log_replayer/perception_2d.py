# Copyright (c) 2023 TIER IV.inc
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

from dataclasses import dataclass
from typing import ClassVar

from perception_eval.evaluation import PerceptionFrameResult
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from driving_log_replayer.criteria import PerceptionCriteria
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase


class Perception2DResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__pass_rate = condition["PassRate"]
        self.__target_cameras = condition["TargetCameras"]
        self.__success = {}
        self.__total = {}
        self.__result = {}
        self.__msg = {}
        for camera_type in self.__target_cameras:
            self.__success[camera_type] = 0
            self.__total[camera_type] = 0
            self.__result[camera_type] = True
            self.__msg[camera_type] = "NotTested"

        self.__criteria = PerceptionCriteria(
            method=condition.get("CriteriaMethod"),
            level=condition.get("CriteriaLevel"),
        )

    def update(self) -> None:
        summary_str = ""
        for camera_type, eval_msg in self.__msg.items():
            summary_str += f" {camera_type}: {eval_msg}"
        if all(self.__result.values()):  # if all camera results are True
            self._success = True
            self._summary = f"Passed:{summary_str}"
        else:
            self._success = False
            self._summary = f"Failed:{summary_str}"

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        header: Header,  # noqa
        map_to_baselink: dict,
        camera_type: str,
    ) -> None:
        self.__total[camera_type] += 1
        result = self.__criteria.get_result(frame)

        if result.is_success():
            self.__success[camera_type] += 1

        test_rate = self.__success[camera_type] / self.__total[camera_type] * 100.0
        self.__result[camera_type] = test_rate >= self.__pass_rate
        self.__msg[
            camera_type
        ] = f"{self.__success[camera_type]} / {self.__total[camera_type]} -> {test_rate:.2f}%"

        out_frame = {
            "CameraType": camera_type,
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        out_frame["PassFail"] = {
            "Result": str(result),
            "Info": [
                {
                    "TP": len(frame.pass_fail_result.tp_object_results),
                    "FP": len(frame.pass_fail_result.fp_object_results),
                    "FN": len(frame.pass_fail_result.fn_objects),
                },
            ],
        }
        self._frame = out_frame
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
