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

from autoware_auto_perception_msgs.msg import TrafficSignal
from autoware_auto_perception_msgs.msg import TrafficSignalArray
from perception_eval.common.object2d import DynamicObject2D
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.metrics import MetricsScore
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer2D
from perception_eval.util.logger_config import configure_logger
import rclpy

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.traffic_light import TrafficLightResult
from driving_log_replayer.traffic_light import TrafficLightScenario

if TYPE_CHECKING:
    from perception_eval.evaluation import PerceptionFrameResult


class TrafficLightEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, TrafficLightScenario, TrafficLightResult)
        self._scenario: TrafficLightScenario
        self.__p_cfg = self._scenario.Evaluation.PerceptionEvaluationConfig
        self.__c_cfg = self._scenario.Evaluation.CriticalObjectFilterConfig
        self.__f_cfg = self._scenario.Evaluation.PerceptionPassFailConfig
        self.__evaluation_task = self.__p_cfg["evaluation_config_dict"]["evaluation_task"]
        self.__p_cfg["evaluation_config_dict"][
            "label_prefix"
        ] = "traffic_light"  # Add a fixed value setting
        self.__p_cfg["evaluation_config_dict"][
            "count_label_number"
        ] = True  # Add a fixed value setting
        self.__camera_type = self.__p_cfg["camera_type"]
        if not self.check_evaluation_task():
            rclpy.shutdown()

        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=self._t4_dataset_paths,
            frame_id=self.__camera_type,
            result_root_directory=Path(
                self._perception_eval_log_path,
                "result",
                "{TIME}",
            ).as_posix(),
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
                target_labels=self.__c_cfg["target_labels"],
            )
        )
        # Pass fail を決めるパラメータ
        self.__frame_pass_fail_config: PerceptionPassFailConfig = PerceptionPassFailConfig(
            evaluator_config=evaluation_config,
            target_labels=self.__f_cfg["target_labels"],
        )
        self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)
        self.__sub_traffic_signals = self.create_subscription(
            TrafficSignalArray,
            "/perception/traffic_light_recognition/traffic_signals",
            self.traffic_signals_cb,
            1,
        )
        self.__skip_counter = 0

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task != "classification2d":
            self.get_logger().error(f"Unexpected evaluation task: {self.__evaluation_task}")
            return False
        return True

    def timer_cb(self) -> None:
        super().timer_cb(register_shutdown_func=self.write_metrics)

    def write_metrics(self) -> None:
        self.save_pkl(self.__evaluator.frame_results)
        self.get_final_result()
        score_dict = {}
        conf_mat_dict = {}
        analyzer = PerceptionAnalyzer2D(self.__evaluator.evaluator_config)
        analyzer.add(self.__evaluator.frame_results)
        score_df, conf_mat_df = analyzer.analyze()
        if score_df is not None:
            score_dict = score_df.to_dict()
        if conf_mat_df is not None:
            conf_mat_dict = conf_mat_df.to_dict()
        final_metrics = {"Score": score_dict, "ConfusionMatrix": conf_mat_dict}
        self._result.set_final_metrics(final_metrics)
        self._result_writer.write_result(self._result)

    def list_dynamic_object_2d_from_ros_msg(
        self,
        unix_time: int,
        signals: list[TrafficSignal],
    ) -> list[DynamicObject2D]:
        estimated_objects: list[DynamicObject2D] = []
        for signal in signals:
            most_probable_light = DLREvaluator.get_most_probable_signal(signal.lights)
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                name=DLREvaluator.get_traffic_light_label_str(most_probable_light),
            )

            estimated_object = DynamicObject2D(
                unix_time=unix_time,
                frame_id=self.__camera_type,
                semantic_score=most_probable_light.confidence,
                semantic_label=label,
                roi=None,
                uuid=str(signal.map_primitive_id),
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def traffic_signals_cb(self, msg: TrafficSignalArray) -> None:
        map_to_baselink = self.lookup_transform(msg.header.stamp)
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
        else:
            estimated_objects: list[DynamicObject2D] = self.list_dynamic_object_2d_from_ros_msg(
                unix_time,
                msg.signals,
            )
            ros_critical_ground_truth_objects = ground_truth_now_frame.objects
            frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
                unix_time=unix_time,
                ground_truth_now_frame=ground_truth_now_frame,
                estimated_objects=estimated_objects,
                ros_critical_ground_truth_objects=ros_critical_ground_truth_objects,
                critical_object_filter_config=self.__critical_object_filter_config,
                frame_pass_fail_config=self.__frame_pass_fail_config,
            )
            self._result.set_frame(
                frame_result,
                self.__skip_counter,
                DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
            )
            self._result_writer.write_result(self._result)

    def get_final_result(self) -> MetricsScore:
        final_metric_score = self.__evaluator.get_scene_result()

        # final result
        logging.info("final metrics result %s", final_metric_score)
        return final_metric_score


@evaluator_main
def main() -> DLREvaluator:
    return TrafficLightEvaluator("traffic_light_evaluator")


if __name__ == "__main__":
    main()
