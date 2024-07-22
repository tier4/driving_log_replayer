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

from autoware_perception_msgs.msg import TrafficLightGroup
from autoware_perception_msgs.msg import TrafficLightGroupArray
from geometry_msgs.msg import TransformStamped
import lanelet2  # noqa
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import distance
from lanelet2.geometry import to2D
from perception_eval.common.object2d import DynamicObject2D
from perception_eval.common.schema import FrameID
from perception_eval.common.transform import TransformDict
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
from driving_log_replayer.lanelet2_util import load_map
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer.traffic_light import FailResultHolder
from driving_log_replayer.traffic_light import get_traffic_light_label_str
from driving_log_replayer.traffic_light import TrafficLightResult
from driving_log_replayer.traffic_light import TrafficLightScenario

if TYPE_CHECKING:
    from perception_eval.evaluation import PerceptionFrameResult


class TrafficLightEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, TrafficLightScenario, TrafficLightResult)
        self._scenario: TrafficLightScenario
        self._result: TrafficLightResult

        self.declare_parameter("map_path", "")
        map_path = Path(
            expandvars(
                self.get_parameter("map_path").get_parameter_value().string_value,
            ),
            "lanelet2_map.osm",
        ).as_posix()
        self.__lanelet_map = load_map(map_path)
        self.fail_result_holder = FailResultHolder(self._perception_eval_log_path)

        self._scenario: TrafficLightScenario
        self._result: TrafficLightResult

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
        self.__camera_type: str = self.__p_cfg["camera_type"]
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
        self.__evaluator = PerceptionEvaluationManager(
            evaluation_config=evaluation_config,
        )
        self.__sub_traffic_signals = self.create_subscription(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            self.traffic_signals_cb,
            1,
        )
        self.__skip_counter = 0

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task != "classification2d":
            self.get_logger().error(
                f"Unexpected evaluation task: {self.__evaluation_task}",
            )
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
        result = analyzer.analyze()
        if result.score is not None:
            score_dict = result.score.to_dict()
        if result.confusion_matrix is not None:
            conf_mat_dict = result.confusion_matrix.to_dict()
        final_metrics = {"Score": score_dict, "ConfusionMatrix": conf_mat_dict}
        self._result.set_final_metrics(final_metrics)
        self.fail_result_holder.save()
        self._result_writer.write_result(self._result)

    def list_dynamic_object_2d_from_ros_msg(
        self,
        unix_time: int,
        traffic_light_groups: list[TrafficLightGroup],
        cam2map: TransformDict,
    ) -> list[DynamicObject2D]:
        estimated_objects: list[DynamicObject2D] = []
        for signal in traffic_light_groups:
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                get_traffic_light_label_str(signal.elements),
            )
            confidence: float = max(signal.elements, key=lambda x: x.confidence).confidence
            signal_pos = self.get_traffic_light_pos(signal.traffic_light_group_id, cam2map)
            # debug self.get_logger().error(f"{signal_pos=}")

            estimated_object = DynamicObject2D(
                unix_time=unix_time,
                frame_id=FrameID.CAM_TRAFFIC_LIGHT,
                semantic_score=confidence,
                semantic_label=label,
                roi=None,
                uuid=str(signal.traffic_light_group_id),
                position=signal_pos,
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def get_traffic_light_pos(
        self,
        traffic_light_uuid: int,
        cam2map: TransformDict,
    ) -> tuple[float, float, float, float]:
        traffic_light_obj = self.__lanelet_map.regulatoryElementLayer.get(traffic_light_uuid)
        light_ls = traffic_light_obj.trafficLights[0]  # とりあえず1個目のline string
        return cam2map.inv().transform((light_ls[0].x, light_ls[0].y, light_ls[0].z))

    def get_traffic_light_pos_and_dist(
        self,
        traffic_light_uuid: str,
        map_to_baselink: TransformStamped,
    ) -> tuple[float, float, float, float]:
        rtn_distance = self.__p_cfg["evaluation_config_dict"]["max_distance"] + 1.0
        try:
            int_uuid = int(traffic_light_uuid)
        except ValueError:
            return (0.0, 0.0, 0.0, rtn_distance)
        else:
            traffic_light_obj = self.__lanelet_map.regulatoryElementLayer.get(int_uuid)
            ego_position = map_to_baselink.transform.translation
            light_ls = traffic_light_obj.trafficLights[0]  # とりあえず1個目のline string
            # 左右の端の位置が入っている。とりあえず大きな差はないとみなして0を取る https://tech.tier4.jp/entry/2021/06/23/160000
            l2d = to2D(light_ls)
            p2d = BasicPoint2d(ego_position.x, ego_position.y)
            return (light_ls[0].x, light_ls[0].y, light_ls[0].z, distance(l2d, p2d))

    def traffic_signals_cb(self, msg: TrafficLightGroupArray) -> None:
        map_to_baselink = self.lookup_transform(msg.stamp)
        unix_time: int = eval_conversions.unix_time_from_ros_timestamp(msg.stamp)
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(unix_time)

        if ground_truth_now_frame is None:
            self.__skip_counter += 1
            return

        cam2map = ground_truth_now_frame.transforms[(FrameID.CAM_TRAFFIC_LIGHT, FrameID.MAP)]

        for obj in ground_truth_now_frame.objects:
            x, y, z, _ = self.get_traffic_light_pos_and_dist(obj.uuid, map_to_baselink)
            # _ replace with dist and then self.get_logger().error(f"{dist=}")
            # transform to camera coordinate system
            position = cam2map.inv().transform((x, y, z))
            obj.set_position(position)

        estimated_objects = self.list_dynamic_object_2d_from_ros_msg(
            unix_time,
            msg.traffic_light_groups,
            cam2map,
        )
        frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
            unix_time=unix_time,
            ground_truth_now_frame=ground_truth_now_frame,
            estimated_objects=estimated_objects,
            critical_object_filter_config=self.__critical_object_filter_config,
            frame_pass_fail_config=self.__frame_pass_fail_config,
        )
        self._result.set_frame(
            frame_result,
            self.__skip_counter,
            DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
        )
        self.fail_result_holder.add_frame(frame_result)
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
