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
from dataclasses import field
from typing import ClassVar

from perception_eval.evaluation import PerceptionFrameResult

from driving_log_replayer.criteria import PerceptionCriteria
from driving_log_replayer.result import EvaluationItem
from driving_log_replayer.result import ResultBase


@dataclass
class Perception(EvaluationItem):
    name: ClassVar[str] = "Perception2D Perception"
    passed_cameras: dict[str, int] = field(default_factory=dict)
    total_cameras: dict[str, int] = field(default_factory=dict)
    success_cameras: dict[str, bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.criteria: PerceptionCriteria = PerceptionCriteria(
            method=self.condition.get("CriteriaMethod"),
            level=self.condition.get("CriteriaLevel"),
        )
        for camera_type in self.condition.get("TargetCameras"):
            self.passed_cameras[camera_type] = 0
            self.total_cameras[camera_type] = 0
            self.success_cameras[camera_type] = True

    def camera_success_str(self, camera_name: str) -> str:
        return "Success" if self.success_cameras.get(camera_name) else "Fail"

    def rate_camera(self, camera_name: str) -> float:
        return (
            0.0
            if self.total_cameras[camera_name] == 0
            else self.passed_cameras[camera_name] / self.total_cameras[camera_name] * 100.0
        )

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        map_to_baselink: dict,
        camera_type: str,
    ) -> dict:
        self.total[camera_type] += 1
        frame_success = "Fail"
        result = self.criteria.get_result(frame)

        if result.is_success():
            self.passed_cameras[camera_type] += 1
            frame_success = "Success"

        self.success_cameras[camera_type] = (
            self.rate_camera(camera_type) >= self.condition["PassRate"]
        )
        self.update()

        return (
            {
                "CameraType": camera_type,
                "Ego": {"TransformStamped": map_to_baselink},
                "FrameName": frame.frame_name,
                "FrameSkip": skip,
                "PassFail": {
                    "Result": {
                        "Total": self.camera_success_str(camera_type),
                        "Frame": frame_success,
                    },
                    "Info": {
                        "TP": len(frame.pass_fail_result.tp_object_results),
                        "FP": len(frame.pass_fail_result.fp_object_results),
                        "FN": len(frame.pass_fail_result.fn_objects),
                    },
                },
            },
        )

    def update(self) -> None:
        tmp_success = True
        tmp_summary = ""
        for camera_name, v in self.success_cameras.items():
            tmp_summary += f"{camera_name}: {self.passed_cameras[camera_name]} / {self.total_cameras[camera_name]} -> {self.rate_camera(camera_name):.2f}% "
            # If even one false is entered, the entire test is false.
            if not v:
                tmp_success = False
        self.success = tmp_success
        tmp_success_str = "Success" if tmp_success else "Fail"
        self.summary = f"{self.name} ({tmp_success_str}): {tmp_summary.rstrip()}"


class Perception2DResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__perception = Perception(condition=condition)

    def update(self) -> None:
        summary_str = f"{self.__perception.summary}"
        if self.__perception.success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        map_to_baselink: dict,
        camera_type: str,
    ) -> None:
        self._frame = self.__perception.set_frame(frame, skip, map_to_baselink, camera_type)
        self.update()

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
