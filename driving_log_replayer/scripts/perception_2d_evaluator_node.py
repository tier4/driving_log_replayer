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
from std_msgs.msg import Header
from tier4_perception_msgs.msg import DetectedObjectsWithFeature
from tier4_perception_msgs.msg import DetectedObjectWithFeature

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
import driving_log_replayer.perception_eval_conversions as eval_conversions
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


class Perception2DEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.check_scenario()
        self.use_t4_dataset()

        self.__result = Perception2DResult(self._condition)

        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=self._t4_dataset_paths,
            frame_id=list(self.__camera_type_dict.keys()),
            result_root_directory=os.path.join(self._perception_eval_log_path, "result", "{TIME}"),
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

    def check_scenario(self) -> None:
        try:
            self.__p_cfg = self._scenario_yaml_obj["Evaluation"]["PerceptionEvaluationConfig"]
            self.__c_cfg = self._scenario_yaml_obj["Evaluation"]["CriticalObjectFilterConfig"]
            self.__f_cfg = self._scenario_yaml_obj["Evaluation"]["PerceptionPassFailConfig"]
            self.__evaluation_task = self.__p_cfg["evaluation_config_dict"]["evaluation_task"]
            self.__p_cfg["evaluation_config_dict"][
                "label_prefix"
            ] = "autoware"  # Add a fixed value setting
            self.__p_cfg["evaluation_config_dict"][
                "count_label_number"
            ] = True  # Add a fixed value setting
            self.__camera_type_dict = self._condition["TargetCameras"]
        except KeyError:
            self.get_logger().error("Scenario format error.")
            rclpy.shutdown()
        if not self.check_evaluation_task():
            rclpy.shutdown()
        if not self.check_camera_type():
            rclpy.shutdown()

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task in ["detection2d", "tracking2d"]:
            return True
        self.get_logger().error(f"Unexpected evaluation task: {self.__evaluation_task}.")
        return False

    def check_camera_type(self) -> bool:
        if isinstance(self.__camera_type_dict, str) or len(self.__camera_type_dict) == 0:
            self.get_logger().error("camera_types is not appropriate.")
            return False
        return True

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
        score_df, conf_mat_df = analyzer.analyze()
        if score_df is not None:
            score_dict = score_df.to_dict()
        if conf_mat_df is not None:
            conf_mat_dict = conf_mat_df.to_dict()
        final_metrics = {"Score": score_dict, "ConfusionMatrix": conf_mat_dict}
        self.__result.set_final_metrics(final_metrics)
        self._result_writer.write(self.__result)

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
            # write result
            self.__result.set_frame(
                frame_result,
                self.__skip_counter[camera_type],
                msg.header,
                DLREvaluator.transform_stamped_with_euler_angle(map_to_baselink),
                camera_type,
            )
            self._result_writer.write(self.__result)

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
