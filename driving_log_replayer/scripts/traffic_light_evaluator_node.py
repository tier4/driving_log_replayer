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
import os
from pathlib import Path
from typing import Dict
from typing import List

from autoware_auto_perception_msgs.msg import TrafficLight
from autoware_auto_perception_msgs.msg import TrafficSignal
from autoware_auto_perception_msgs.msg import TrafficSignalArray
from geometry_msgs.msg import TransformStamped
from perception_eval.common.object2d import DynamicObject2D
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation import PerceptionFrameResult
from perception_eval.evaluation.metrics import MetricsScore
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer2D
from perception_eval.util.logger_config import configure_logger
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener
import yaml

from driving_log_replayer.node_common import transform_stamped_with_euler_angle
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.result import PickleWriter
from driving_log_replayer.result import ResultBase
from driving_log_replayer.result import ResultWriter


def get_label(light: TrafficLight) -> str:
    if light.color == TrafficLight.RED:
        return "red"
    elif light.color == TrafficLight.AMBER:
        return "yellow"
    elif light.color == TrafficLight.GREEN:
        return "green"
    else:
        return "unknown"


def get_most_probable_signal(
    lights: List[TrafficLight],
) -> TrafficLight:
    highest_probability = 0.0
    highest_light = None
    for light in lights:
        if light.confidence >= highest_probability:
            highest_probability = light.confidence
            highest_light = light
    return highest_light


class TrafficLightResult(ResultBase):
    def __init__(self, condition: Dict):
        super().__init__()
        self.__pass_rate = condition["PassRate"]
        self.__success = 0
        self.__total = 0

    def update(self):
        test_rate = 0.0 if self.__total == 0 else self.__success / self.__total * 100.0
        success = test_rate >= self.__pass_rate
        summary_str = f"{self.__success} / {self.__total } -> {test_rate:.2f}%"

        if success:
            self._success = True
            self._summary = f"Passed: {summary_str}"
        else:
            self._success = False
            self._summary = f"Failed: {summary_str}"

    def add_frame(
        self,
        frame: PerceptionFrameResult,
        skip: int,
        header: Header,  # noqa
        map_to_baselink: Dict,
    ):
        self.__total += 1
        has_objects = True
        # OptionalでNoneが入る場合と、空配列の場合の2種類ありそうなので、is None判定ではなくnotで判定する
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
                }
            ],
        }
        self._frame = out_frame
        self.update()

    def add_final_metrics(self, final_metrics: Dict):
        self._frame = {"FinalScore": final_metrics}


class TrafficLightEvaluator(Node):
    def __init__(self):
        super().__init__("traffic_light_evaluator")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("result_json_path", "")
        self.declare_parameter("t4_dataset_path", "")
        self.declare_parameter("result_archive_path", "")

        self.__timer_group = MutuallyExclusiveCallbackGroup()
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self, spin_thread=True)

        scenario_path = os.path.expandvars(
            self.get_parameter("scenario_path").get_parameter_value().string_value
        )
        self.__scenario_yaml_obj = None
        with open(scenario_path, "r") as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)
        self.__result_json_path = os.path.expandvars(
            self.get_parameter("result_json_path").get_parameter_value().string_value
        )
        result_archive_path = Path(
            os.path.expandvars(
                self.get_parameter("result_archive_path").get_parameter_value().string_value
            )
        )
        result_archive_path.mkdir(exist_ok=True)

        self.__pkl_path = result_archive_path.joinpath("scene_result.pkl").as_posix()
        self.__t4_dataset_paths = [
            os.path.expandvars(
                self.get_parameter("t4_dataset_path").get_parameter_value().string_value
            )
        ]
        self.__perception_eval_log_path = result_archive_path.parent.joinpath(
            "perception_eval_log"
        ).as_posix()

        try:
            self.__condition = self.__scenario_yaml_obj["Evaluation"]["Conditions"]
            self.__result = TrafficLightResult(self.__condition)

            self.__result_writer = ResultWriter(
                self.__result_json_path, self.get_clock(), self.__condition
            )

            p_cfg = self.__scenario_yaml_obj["Evaluation"]["PerceptionEvaluationConfig"]
            c_cfg = self.__scenario_yaml_obj["Evaluation"]["CriticalObjectFilterConfig"]
            f_cfg = self.__scenario_yaml_obj["Evaluation"]["PerceptionPassFailConfig"]

            self.__camera_type = p_cfg["camera_type"]
            p_cfg["evaluation_config_dict"][
                "label_prefix"
            ] = "traffic_light"  # Add a fixed value setting
            p_cfg["evaluation_config_dict"][
                "count_label_number"
            ] = True  # Add a fixed value setting

            evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
                dataset_paths=self.__t4_dataset_paths,
                frame_id=self.__camera_type,
                result_root_directory=os.path.join(
                    self.__perception_eval_log_path, "result", "{TIME}"
                ),
                evaluation_config_dict=p_cfg["evaluation_config_dict"],
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
                    target_labels=c_cfg["target_labels"],
                )
            )
            # Pass fail を決めるパラメータ
            self.__frame_pass_fail_config: PerceptionPassFailConfig = PerceptionPassFailConfig(
                evaluator_config=evaluation_config,
                target_labels=f_cfg["target_labels"],
            )
            self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)
            self.__sub_traffic_signals = self.create_subscription(
                TrafficSignalArray,
                "/perception/traffic_light_recognition/traffic_signals",
                self.traffic_signals_cb,
                1,
            )

            self.__current_time = Time().to_msg()
            self.__prev_time = Time().to_msg()

            self.__counter = 0
            self.__timer = self.create_timer(
                1.0,
                self.timer_cb,
                callback_group=self.__timer_group,
                clock=Clock(clock_type=ClockType.SYSTEM_TIME),
            )  # wall timer
            self.__skip_counter = 0
        except KeyError:
            # Immediate termination if the scenario does not contain the required items and is incompatible.
            self.get_logger().error("Scenario format error.")
            rclpy.shutdown()

    def timer_cb(self):
        self.__current_time = self.get_clock().now().to_msg()
        # self.get_logger().error(f"time: {self.__current_time.sec}.{self.__current_time.nanosec}")
        if self.__current_time.sec > 0:
            if self.__current_time == self.__prev_time:
                self.__counter += 1
            else:
                self.__counter = 0
            self.__prev_time = self.__current_time
            if self.__counter >= 5:
                self.__pickle_writer = PickleWriter(self.__pkl_path)
                self.__pickle_writer.dump(self.__evaluator.frame_results)
                self.get_final_result()
                analyzer = PerceptionAnalyzer2D(self.__evaluator.evaluator_config)
                analyzer.add(self.__evaluator.frame_results)
                score_df, conf_mat_df = analyzer.analyze()
                score_dict = {}
                conf_mat_dict = {}
                if score_df is not None:
                    score_dict = score_df.to_dict()
                if conf_mat_df is not None:
                    conf_mat_dict = conf_mat_df.to_dict()
                final_metrics = {"Score": score_dict, "ConfusionMatrix": conf_mat_dict}
                self.__result.add_final_metrics(final_metrics)
                self.__result_writer.write(self.__result)
                self.__result_writer.close()
                rclpy.shutdown()

    def list_dynamic_object_2d_from_ros_msg(
        self, unix_time: int, signals: List[TrafficSignal]
    ) -> List[DynamicObject2D]:
        estimated_objects: List[DynamicObject2D] = []
        for signal in signals:
            most_probable_light = get_most_probable_signal(signal.lights)
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                name=get_label(most_probable_light)
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

    def traffic_signals_cb(self, msg: TrafficSignalArray):
        try:
            map_to_baselink = self.__tf_buffer.lookup_transform(
                "map", "base_link", msg.header.stamp, Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().info(f"Could not transform map to baselink: {ex}")
            map_to_baselink = TransformStamped()
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        # 現frameに対応するGround truthを取得
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)
        if ground_truth_now_frame is None:
            self.__skip_counter += 1
        else:
            # self.get_logger().error(
            #     f"debug: get ground truth: {self.__current_time.sec}.{self.__current_time.nanosec}"
            # )
            estimated_objects: List[DynamicObject2D] = self.list_dynamic_object_2d_from_ros_msg(
                unix_time, msg.signals
            )
            # self.get_logger().error(
            #     f"debug: get dynamic object 2d: {self.__current_time.sec}.{self.__current_time.nanosec}"
            # )
            ros_critical_ground_truth_objects = ground_truth_now_frame.objects
            # critical_object_filter_configと、frame_pass_fail_configこの中で動的に変えても良い。
            # 動的に変える条件をかけるようになるまでは、初期化時に一括設定

            frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
                unix_time=unix_time,
                ground_truth_now_frame=ground_truth_now_frame,
                estimated_objects=estimated_objects,
                ros_critical_ground_truth_objects=ros_critical_ground_truth_objects,
                critical_object_filter_config=self.__critical_object_filter_config,
                frame_pass_fail_config=self.__frame_pass_fail_config,
            )
            # self.get_logger().error(
            #     f"debug: get frame result: {self.__current_time.sec}.{self.__current_time.nanosec}"
            # )
            # write result
            self.__result.add_frame(
                frame_result,
                self.__skip_counter,
                msg.header,
                transform_stamped_with_euler_angle(map_to_baselink),
            )
            self.__result_writer.write(self.__result)

    def get_final_result(self) -> MetricsScore:
        final_metric_score = self.__evaluator.get_scene_result()

        # final result
        logging.info("final metrics result %s", final_metric_score)
        return final_metric_score


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    traffic_light_evaluator = TrafficLightEvaluator()
    executor.add_node(traffic_light_evaluator)
    executor.spin()
    traffic_light_evaluator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
