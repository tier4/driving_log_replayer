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


class PerceptionResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__pass_rate = condition["PassRate"]
        self.__success = 0
        self.__total = 0

        self.__criteria = PerceptionCriteria(
            method=condition.get("CriteriaMethod"),
            level=condition.get("CriteriaLevel"),
        )

    def update(self) -> None:
        test_rate = 0.0 if self.__total == 0 else self.__success / self.__total * 100.0
        success = test_rate >= self.__pass_rate
        summary_str = f"{self.__success} / {self.__total } -> {test_rate:.2f}%"

        if success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def set_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        header: Header,
        map_to_baselink: dict,
    ) -> tuple[MarkerArray, MarkerArray]:
        self.__total += 1
        result = self.__criteria.get_result(frame)

        if result.is_success():
            self.__success += 1

        out_frame = {
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
        marker_ground_truth = MarkerArray()
        color_success = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3)

        for cnt, obj in enumerate(frame.frame_ground_truth.objects, start=1):
            bbox, uuid = eval_conversions.object_state_to_ros_box_and_uuid(
                obj.state,
                header,
                "ground_truth",
                cnt,
                color_success,
                obj.uuid,
            )
            marker_ground_truth.markers.append(bbox)
            marker_ground_truth.markers.append(uuid)

        marker_results = eval_conversions.pass_fail_result_to_ros_points_array(
            frame.pass_fail_result,
            header,
        )

        self._frame = out_frame
        self.update()
        return marker_ground_truth, marker_results

    def set_final_metrics(self, final_metrics: dict) -> None:
        self._frame = {"FinalScore": final_metrics}
