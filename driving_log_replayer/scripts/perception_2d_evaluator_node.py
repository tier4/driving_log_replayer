#!/usr/bin/env python3

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

import logging
from pathlib import Path
from typing import TYPE_CHECKING

from perception_eval.common.object2d import DynamicObject2D
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.metrics import MetricsScore
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer2D
from perception_eval.util.logger_config import configure_logger
import rclpy
from tier4_perception_msgs.msg import DetectedObjectsWithFeature
from tier4_perception_msgs.msg import DetectedObjectWithFeature

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.perception_2d import Perception2DResult
from driving_log_replayer.perception_2d import Perception2DScenario
import driving_log_replayer.perception_eval_conversions as eval_conversions

if TYPE_CHECKING:
    from perception_eval.evaluation import PerceptionFrameResult


class Perception2DEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, Perception2DScenario, Perception2DResult)
        self._scenario: Perception2DScenario
        self._result: Perception2DResult

        self.__p_cfg = self._scenario.Evaluation.PerceptionEvaluationConfig
        self.__c_cfg = self._scenario.Evaluation.CriticalObjectFilterConfig
        self.__f_cfg = self._scenario.Evaluation.PerceptionPassFailConfig
        self.__evaluation_task = self.__p_cfg["evaluation_config_dict"]["evaluation_task"]
        self.__p_cfg["evaluation_config_dict"][
            "label_prefix"
        ] = "autoware"  # Add a fixed value setting
        self.__p_cfg["evaluation_config_dict"][
            "count_label_number"
        ] = True  # Add a fixed value setting
        self.__camera_type_dict = self._scenario.Evaluation.Conditions.TargetCameras
        if not self.check_evaluation_task():
            rclpy.shutdown()

        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=self._t4_dataset_paths,
            frame_id=list(self.__camera_type_dict.keys()),
            result_root_directory=Path(self._perception_eval_log_path)
            .joinpath("result", "{TIME}")
            .as_posix(),
            evaluation_config_dict=self.__p_cfg["evaluation_config_dict"],
            load_raw_data=False,
        )
        _ = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
        )
        # どれを注目物体とするかのparam
        self.__critical_object_filter_config: CriticalObjectFilterConfig = (
            CriticalObjectFilterConfig(
                evaluator_config=evaluation_config,
                ignore_attributes=self.__c_cfg["ignore_attributes"],
                target_labels=self.__c_cfg["target_labels"],
            )
        )
        # Pass fail を決めるパラメータ
        self.__frame_pass_fail_config: PerceptionPassFailConfig = PerceptionPassFailConfig(
            evaluator_config=evaluation_config,
            target_labels=self.__f_cfg["target_labels"],
            matching_threshold_list=self.__f_cfg["matching_threshold_list"],
        )
        self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)

        self.__subscribers = {}
        self.__skip_counter = {}
        for camera_type, camera_no in self.__camera_type_dict.items():
            self.__subscribers[camera_type] = self.create_subscription(
                DetectedObjectsWithFeature,
                self.get_topic_name(camera_no),
                lambda msg, local_type=camera_type: self.detected_objs_cb(msg, local_type),
                1,
            )
            self.__skip_counter[camera_type] = 0

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task in ["detection2d", "tracking2d"]:
            return True
        self.get_logger().error(f"Unexpected evaluation task: {self.__evaluation_task}.")
        return False

    def get_topic_name(self, camera_no: int) -> str:
        if self.__evaluation_task == "detection2d":
            return f"/perception/object_recognition/detection/rois{camera_no}"
        return f"/perception/object_recognition/detection/tracked/rois{camera_no}"  # tracking2d

    def timer_cb(self) -> None:
        super().timer_cb(register_shutdown_func=self.write_metrics)

    def write_metrics(self) -> None:
        self.save_pkl(self.__evaluator.frame_results)
        self.get_final_result()
        score_dict = {}
        conf_mat_dict = {}
        analyzer = PerceptionAnalyzer2D(self.__evaluator.evaluator_config)
        analyzer.add(self.__evaluator.frame_results)
        result = analyzer.analyze()
        if result.score is not None:
            score_dict = result.score.to_dict()
        if result.confusion_matrix is not None:
            conf_mat_dict = result.confusion_matrix.to_dict()
        final_metrics = {"Score": score_dict, "ConfusionMatrix": conf_mat_dict}
        self._result.set_final_metrics(final_metrics)
        self._result_writer.write_result(self._result)

    def list_dynamic_object_2d_from_ros_msg(
        self,
        unix_time: int,
        feature_objects: list[DetectedObjectWithFeature],
        camera_type: str,
    ) -> list[DynamicObject2D]:
        estimated_objects: list[DynamicObject2D] = []
        for perception_object in feature_objects:
            most_probable_classification = DLREvaluator.get_most_probable_classification(
                perception_object.object.classification,
            )
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                name=DLREvaluator.get_perception_label_str(most_probable_classification),
            )
            obj_roi = perception_object.feature.roi
            roi = obj_roi.x_offset, obj_roi.y_offset, obj_roi.width, obj_roi.height

            estimated_object = DynamicObject2D(
                unix_time=unix_time,
                frame_id=camera_type,
                semantic_score=most_probable_classification.probability,
                semantic_label=label,
                roi=roi,
                uuid=None,
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def detected_objs_cb(self, msg: DetectedObjectsWithFeature, camera_type: str) -> None:
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        # 現frameに対応するGround truthを取得
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)
        if ground_truth_now_frame is None:
            self.__skip_counter[camera_type] += 1
        else:
            estimated_objects: list[DynamicObject2D] = self.list_dynamic_object_2d_from_ros_msg(
                unix_time,
                msg.feature_objects,
                camera_type,
            )

            frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
                unix_time=unix_time,
                ground_truth_now_frame=ground_truth_now_frame,
                estimated_objects=estimated_objects,
                critical_object_filter_config=self.__critical_object_filter_config,
                frame_pass_fail_config=self.__frame_pass_fail_config,
            )
            # write result
            self._result.set_frame(
                frame_result,
                self.__skip_counter[camera_type],
                DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
                camera_type,
            )
            self._result_writer.write_result(self._result)

    def get_final_result(self) -> MetricsScore:
        final_metric_score = self.__evaluator.get_scene_result()

        # final result
        logging.info("final metrics result %s", final_metric_score)
        return final_metric_score


@evaluator_main
def main() -> DLREvaluator:
    return Perception2DEvaluator("perception_2d_evaluator")


if __name__ == "__main__":
    main()
