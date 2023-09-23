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

from perception_eval.evaluation import PerceptionFrameResult
from std_msgs.msg import Header

from driving_log_replayer.result import ResultBase


class TrafficLightResult(ResultBase):
    def __init__(self, condition: dict) -> None:
        super().__init__()
        self.__pass_rate = condition["PassRate"]
        self.__success = 0
        self.__total = 0

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
        header: Header,  # noqa
        map_to_baselink: dict,
    ) -> None:
        self.__total += 1
        has_objects = True
        if (
            frame.pass_fail_result.tp_object_results == []
            and frame.pass_fail_result.fp_object_results == []
            and frame.pass_fail_result.fn_objects == []
        ):
            has_objects = False

        success = (
            "Success"
            if frame.pass_fail_result.get_fail_object_num() == 0 and has_objects
            else "Fail"
        )
        if success == "Success":
            self.__success += 1
        out_frame = {
            "Ego": {"TransformStamped": map_to_baselink},
            "FrameName": frame.frame_name,
            "FrameSkip": skip,
        }
        out_frame["PassFail"] = {
            "Result": success,
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
