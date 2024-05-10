#!/usr/bin/env python3

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.ground_segmentation import GroundSegmentationResult
from driving_log_replayer.ground_segmentation import GroundSegmentationScenario
import driving_log_replayer.perception_eval_conversions as eval_conversions

from driving_log_replayer_msgs.msg import GroundSegmentationEvalResult
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

from typing import List, Dict
import ros2_numpy
import numpy as np

import json
import os

from scipy.spatial import cKDTree


class GroundSegmentationEvaluator(DLREvaluator):
    CLOUD_DIM = 6

    ## label
    ## 6 : ground , 7 : obstacle

    def __init__(self, name: str) -> None:
        super().__init__(name, GroundSegmentationScenario, GroundSegmentationResult)

        self.get_logger().info(f"{len(self._t4_dataset_paths)}")
        for path in self._t4_dataset_paths:
            self.get_logger().info(path)

        sample_data = json.load(
            open(os.path.join(self._t4_dataset_paths[0], "annotation", "sample_data.json"), "r")
        )
        sample_data = list(filter(lambda d: d["filename"].split(".")[-2] == "pcd", sample_data))

        self.ground_truth: Dict[int, np.ndarray] = {}
        for i in range(len(sample_data)):
            pcd_file_path = os.path.join(self._t4_dataset_paths[0], sample_data[i]["filename"])
            raw_points = np.fromfile(pcd_file_path, dtype=np.float32)
            points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))
            self.ground_truth[int(sample_data[i]["timestamp"])] = points

        self.__sub_pointcloud = self.create_subscription(
            PointCloud2,
            "/perception/obstacle_segmentation/single_frame/pointcloud",
            self.pointcloud_cb,
            qos_profile_sensor_data,
        )

    def pointcloud_cb(self, msg: PointCloud2):
        unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)
        gt_frame_ts = self.__get_gt_frame_ts(unix_time=unix_time)

        if gt_frame_ts < 0:
            return
        
        self._logger.info(f"gt frame ts : {gt_frame_ts}")
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
        tp_fn = np.count_nonzero(gt_frame_cloud[:, 5] == 6)
        fp_tn = np.count_nonzero(gt_frame_cloud[:, 5] == 7)
        TN: int = 0
        FN: int = 0
        for p in pointcloud:
            _, idx = kdtree.query(p, k=1)
            if gt_frame_cloud[idx][5] == 6:
                FN += 1
            elif gt_frame_cloud[idx][5] == 7:
                TN += 1
        TP = tp_fn - FN
        FP = fp_tn - TN

        self._logger.info(f"TP {TP}, FP {FP}, TN {TN}, FN {FN}")

        metrics_list = self.__compute_metrics(TP, FP, TN, FN)

        frame_result = GroundSegmentationEvalResult()
        frame_result.tp = TP
        frame_result.fp = FP
        frame_result.tn = TN
        frame_result.fn = FN
        frame_result.precision = metrics_list[0]
        frame_result.recall = metrics_list[1]
        frame_result.specificity = metrics_list[2]
        frame_result.f1_score = metrics_list[3]

        self._result.set_frame(frame_result)
        self._result_writer.write_result(self._result)

    def __get_gt_frame_ts(self, unix_time: int) -> int:
        min_diff: int = abs(unix_time - int(list(self.ground_truth.keys())[0]))
        ret_ts: int = int(list(self.ground_truth.keys())[0])
        for i in range(1, len(self.ground_truth)):
            sample_ts = int(list(self.ground_truth.keys())[i])
            diff_time = abs(unix_time - sample_ts)
            # self._logger.warn(f'keys : {sample_ts}')
            if diff_time < min_diff:
                min_diff = diff_time
                ret_ts = sample_ts

        if min_diff > 75000:
            self._logger.warn(f"time diff {min_diff} is too big")
            return -1

        return ret_ts

    def __compute_metrics(self, tp: int, fp: int, tn: int, fn: int) -> List[float]:
        precision = float(tp) / float(tp + fp + 1e-5)
        recall = float(tp) / float(tp + fn + 1e-5)
        specificity = float(tn) / float(tn + fp + 1e-5)
        f1_score = 2 * (precision * recall) / (precision + recall + 1e-5)
        return [precision, recall, specificity, f1_score]


@evaluator_main
def main() -> DLREvaluator:
    return GroundSegmentationEvaluator("ground_segmentation_evaluator")


if __name__ == "__main__":
    main()
