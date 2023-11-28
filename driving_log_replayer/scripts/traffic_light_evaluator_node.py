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
from os.path import expandvars
from pathlib import Path
from typing import TYPE_CHECKING

from autoware_perception_msgs.msg import TrafficSignal
from autoware_perception_msgs.msg import TrafficSignalArray
from perception_eval.common.object2d import DynamicObject2D
from perception_eval.common.schema import FrameID
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.metrics import MetricsScore
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer2D
from perception_eval.util.logger_config import configure_logger
import rclpy
from simple_lanelet_loader.traffic_light_loader import TrafficLightLoader

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.traffic_light import FailResultHolder
from driving_log_replayer.traffic_light import TrafficLightResult

if TYPE_CHECKING:
    from perception_eval.evaluation import PerceptionFrameResult


class TrafficLightEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()
        self.use_t4_dataset()
        self.use_map_interface()

        self.__result = TrafficLightResult(self._condition)

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

    def check_scenario(self) -> None:
        try:
            self.__p_cfg = self._scenario_yaml_obj["Evaluation"]["PerceptionEvaluationConfig"]
            self.__c_cfg = self._scenario_yaml_obj["Evaluation"]["CriticalObjectFilterConfig"]
            self.__f_cfg = self._scenario_yaml_obj["Evaluation"]["PerceptionPassFailConfig"]
            self.__evaluation_task = self.__p_cfg["evaluation_config_dict"]["evaluation_task"]
            self.__p_cfg["evaluation_config_dict"][
                "label_prefix"
            ] = "traffic_light"  # Add a fixed value setting
            self.__p_cfg["evaluation_config_dict"][
                "count_label_number"
            ] = True  # Add a fixed value setting
            self.__camera_type = self.__p_cfg["camera_type"]
        except KeyError:
            self.get_logger().error("Scenario format error.")
            rclpy.shutdown()
        if not self.check_evaluation_task():
            rclpy.shutdown()

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task != "classification2d":
            self.get_logger().error(f"Unexpected evaluation task: {self.__evaluation_task}")
            return False
        return True

    def use_map_interface(self) -> None:
        self.declare_parameter("map_path", "")
        map_path = Path(
            expandvars(
                self.get_parameter("map_path").get_parameter_value().string_value,
            ),
            "lanelet2_map.osm",
        )
        self.__traffic_light_obj = TrafficLightLoader(map_path)
        self.__use_regulatory_element: bool = True
        self.fail_result_holder = FailResultHolder(self._perception_eval_log_path)

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
        self.__result.set_final_metrics(final_metrics)
        self.fail_result_holder.save()
        self._result_writer.write_result(self.__result)

    def list_dynamic_object_2d_from_ros_msg(
        self,
        unix_time: int,
        signals: list[TrafficSignal],
    ) -> list[DynamicObject2D]:
        estimated_objects: list[DynamicObject2D] = []
        for i, signal in enumerate(signals):
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                DLREvaluator.get_traffic_light_label_str(signal.elements),
            )
            confidence: float = max(signal.elements, key=lambda x: x.confidence)
            logging.info(f"Est {[i]}: {(signal.traffic_signal_id, label.name)}")  # noqa

            estimated_object = DynamicObject2D(
                unix_time=unix_time,
                frame_id=FrameID.TRAFFIC_LIGHT
                if self.__use_regulatory_element
                else self.__camera_type,
                semantic_score=confidence,
                semantic_label=label,
                roi=None,
                uuid=str(signal.traffic_signal_id),
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def traffic_signals_cb(self, msg: TrafficSignalArray) -> None:
        map_to_baselink = self.lookup_transform(msg.stamp)
        unix_time: int = eval_conversions.unix_time_from_ros_timestamp(msg.stamp)
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
            return

        # extract all traffic lights closer than 202[m]
        # TODO: avoid using magic number
        max_distance_threshold = 202.0  # [m]
        filtered_gt_objects = []
        valid_gt_distances = []
        for obj in ground_truth_now_frame.objects:
            ego_position = map_to_baselink.transform.translation
            distance_to_gt = self.__traffic_light_obj.get_distance_to_traffic_light_group(
                obj.uuid,
                [ego_position.x, ego_position.y, ego_position.z],
            )
            if distance_to_gt is not None and distance_to_gt < max_distance_threshold:
                filtered_gt_objects.append(obj)
                valid_gt_distances.append(distance_to_gt)

        estimated_objects = self.list_dynamic_object_2d_from_ros_msg(unix_time, msg.signals)
        filtered_est_objects = []
        valid_est_distances = []
        for obj in estimated_objects:
            ego_position = map_to_baselink.transform.translation
            distance_to_est = self.__traffic_light_obj.get_distance_to_traffic_light_group(
                obj.uuid,
                [ego_position.x, ego_position.y, ego_position.z],
            )
            if distance_to_est is not None and distance_to_est < max_distance_threshold:
                filtered_est_objects.append(obj)
                valid_est_distances.append(distance_to_est)

        if len(filtered_gt_objects) == 0 and len(filtered_est_objects) == 0:
            self.__skip_counter += 1
            return
        ground_truth_now_frame.objects = filtered_gt_objects
        ros_critical_ground_truth_objects = filtered_gt_objects
        logging.info(
            f"GTs: {[(obj.uuid, f'{dist} [m]') for obj, dist in zip(filtered_gt_objects, valid_gt_distances)]}, "  # noqa
            f"ESTs: {[(obj.uuid, f'{dist} [m]') for obj, dist in zip(estimated_objects, valid_est_distances)]}"  # noqa
        )
        frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
            unix_time=unix_time,
            ground_truth_now_frame=ground_truth_now_frame,
            estimated_objects=filtered_est_objects,
            ros_critical_ground_truth_objects=ros_critical_ground_truth_objects,
            critical_object_filter_config=self.__critical_object_filter_config,
            frame_pass_fail_config=self.__frame_pass_fail_config,
        )
        logging.info(
            f"TP: {len(frame_result.pass_fail_result.tp_object_results)}, "  # noqa
            f"FP: {len(frame_result.pass_fail_result.fp_object_results)}, "
            f"FN: {len(frame_result.pass_fail_result.fn_objects)}",
        )
        self.__result.set_frame(
            frame_result,
            self.__skip_counter,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
        )
        self.fail_result_holder.add_frame(frame_result)
        self._result_writer.write_result(self.__result)

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
