#!/usr/bin/env python3

# Copyright (c) 2024 TierIV.inc
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

import json
from pathlib import Path

import numpy as np
import message_filters
from rclpy.qos import qos_profile_sensor_data
import ros2_numpy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.ground_segmentation import GroundSegmentationResult
from driving_log_replayer.ground_segmentation import GroundSegmentationScenario
from driving_log_replayer.ground_segmentation import Condition
import driving_log_replayer.perception_eval_conversions as eval_conversions
from driving_log_replayer_msgs.msg import GroundSegmentationEvalResult


class GroundSegmentationEvaluator(DLREvaluator):
    CLOUD_DIM = 6
    GROUND_LABEL = 6
    OBSTACLE_LABEL = 7
    TS_DIFF_THRESH = 75000

    def __init__(self, name: str) -> None:
        super().__init__(name, GroundSegmentationScenario, GroundSegmentationResult)

        eval_condition: Condition = self._scenario.Evaluation.Conditions

        if eval_condition.Method == "annotated_pcd":
            # pcd eval mode
            sample_data_path = Path(self._t4_dataset_paths[0], "annotation", "sample_data.json")
            sample_data = json.load(sample_data_path.open())
            sample_data = list(filter(lambda d: d["filename"].split(".")[-2] == "pcd", sample_data))

            self.ground_truth: dict[int, np.ndarray] = {}
            for i in range(len(sample_data)):
                pcd_file_path = Path(
                    self._t4_dataset_paths[0], sample_data[i]["filename"]
                ).as_posix()
                raw_points = np.fromfile(pcd_file_path, dtype=np.float32)
                points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))
                self.ground_truth[int(sample_data[i]["timestamp"])] = points

            self.__sub_pointcloud = self.create_subscription(
                PointCloud2,
                "/perception/obstacle_segmentation/single_frame/pointcloud",
                self.pointcloud_cb,
                qos_profile_sensor_data,
            )
        elif eval_condition.Method == "annotated_rosbag":
            # rosbag (AWSIM) eval mode

            self.__sub_gt_cloud = message_filters.Subscriber(
                self,
                PointCloud2,
                "/sensing/lidar/concatenated/pointcloud",
                qos_profile=qos_profile_sensor_data,
            )
            self.__sub_eval_target_cloud = message_filters.Subscriber(
                self,
                PointCloud2,
                "/perception/obstacle_segmentation/single_frame/pointcloud",
                qos_profile=qos_profile_sensor_data,
            )
            self.__sync_sub = message_filters.TimeSynchronizer(
                [self.__sub_gt_cloud, self.__sub_eval_target_cloud], 1000
            )
            self.__sync_sub.registerCallback(self.annotated_rosbag_eval_cb)
        else:
            raise ValueError(
                'The "Method" field must be set to either "annotated_rosbag" or "annotated_pcd"'
            )

    def pointcloud_cb(self, msg: PointCloud2) -> None:
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        gt_frame_ts = self.__get_gt_frame_ts(unix_time=unix_time)

        if gt_frame_ts < 0:
            return

        # get ground truth pointcloud in this frame
        # construct kd-tree from gt cloud
        gt_frame_cloud: np.ndarray = self.ground_truth[gt_frame_ts]
        kdtree = cKDTree(gt_frame_cloud[:, 0:3])

        # convert ros2 pointcloud to numpy
        numpy_pcd = ros2_numpy.numpify(msg)
        pointcloud = np.zeros((numpy_pcd.shape[0], 3))
        pointcloud[:, 0] = numpy_pcd["x"]
        pointcloud[:, 1] = numpy_pcd["y"]
        pointcloud[:, 2] = numpy_pcd["z"]

        # count TP+FN, TN+FP
        tp_fn = np.count_nonzero(gt_frame_cloud[:, 5] == self.GROUND_LABEL)
        fp_tn = np.count_nonzero(gt_frame_cloud[:, 5] == self.OBSTACLE_LABEL)
        tn: int = 0
        fn: int = 0
        for p in pointcloud:
            _, idx = kdtree.query(p, k=1)
            if gt_frame_cloud[idx][5] == self.GROUND_LABEL:
                fn += 1
            elif gt_frame_cloud[idx][5] == self.OBSTACLE_LABEL:
                tn += 1
        tp = tp_fn - fn
        fp = fp_tn - tn

        self.get_logger().info(f"TP {tp}, FP {fp}, TN {tn}, FN {fn}")

        metrics_list = self.__compute_metrics(tp, fp, tn, fn)

        frame_result = GroundSegmentationEvalResult()
        frame_result.tp = tp
        frame_result.fp = fp
        frame_result.tn = tn
        frame_result.fn = fn
        frame_result.accuracy = metrics_list[0]
        frame_result.precision = metrics_list[1]
        frame_result.recall = metrics_list[2]
        frame_result.specificity = metrics_list[3]
        frame_result.f1_score = metrics_list[4]

        self._result.set_frame(frame_result)
        self._result_writer.write_result(self._result)

    def annotated_rosbag_eval_cb(
        self, gt_cloud_msg: PointCloud2, eval_target_cloud_msg: PointCloud2
    ):
        np_gt_cloud: np.array = ros2_numpy.numpify(gt_cloud_msg)
        np_target_cloud: np.array = ros2_numpy.numpify(eval_target_cloud_msg)

        # guard
        if (
            "entity_id" not in np_gt_cloud.dtype.fields
            or "entity_id" not in np_target_cloud.dtype.fields
        ):
            return

        tp_fn = np.count_nonzero(np_gt_cloud["entity_id"] == 1)
        tn_fp = np_gt_cloud.size - tp_fn
        fn = np.count_nonzero(np_target_cloud["entity_id"] == 1)
        tn = np_target_cloud.size - fn

        tp = tp_fn - fn
        fp = tn_fp - tn
        self.get_logger().info(f"TP+FN = {tp_fn}, TN+FP = {tn_fp}")
        self.get_logger().info(f"TP {tp}, FP {fp}, TN {tn}, FN {fn}")
        metrics_list = self.__compute_metrics(tp, fp, tn, fn)
        frame_result = GroundSegmentationEvalResult()
        frame_result.tp = tp
        frame_result.fp = fp
        frame_result.tn = tn
        frame_result.fn = fn
        frame_result.accuracy = metrics_list[0]
        frame_result.precision = metrics_list[1]
        frame_result.recall = metrics_list[2]
        frame_result.specificity = metrics_list[3]
        frame_result.f1_score = metrics_list[4]

        self._result.set_frame(frame_result)
        self._result_writer.write_result(self._result)

    def eval_result_cb(self, msg: GroundSegmentationEvalResult):
        self._result.set_frame(msg)
        self._result_writer.write_result(self._result)

    def __get_gt_frame_ts(self, unix_time: int) -> int:
        ts_itr = iter(self.ground_truth.keys())
        ret_ts: int = int(next(ts_itr))
        min_diff: int = abs(unix_time - ret_ts)

        for _ in range(1, len(self.ground_truth)):
            sample_ts = next(ts_itr)
            diff_time = abs(unix_time - sample_ts)
            if diff_time < min_diff:
                min_diff = diff_time
                ret_ts = sample_ts

        if min_diff > self.TS_DIFF_THRESH:
            self.get_logger().warn("time diff is too big")
            return -1

        return ret_ts

    def __compute_metrics(self, tp: int, fp: int, tn: int, fn: int) -> list[float]:
        eps = 1e-10
        accuracy = float(tp + tn) / float(tp + fp + tn + fn + eps)
        precision = float(tp) / float(tp + fp + eps)
        recall = float(tp) / float(tp + fn + eps)
        specificity = float(tn) / float(tn + fp + eps)
        f1_score = 2 * (precision * recall) / (precision + recall + eps)
        return [accuracy, precision, recall, specificity, f1_score]


@evaluator_main
def main() -> DLREvaluator:
    return GroundSegmentationEvaluator("ground_segmentation_evaluator")


if __name__ == "__main__":
    main()
