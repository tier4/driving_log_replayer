#!/usr/bin/env python3

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.ground_segmentation import GroundSegmentationResult
from driving_log_replayer.ground_segmentation import GroundSegmentationScenario
import driving_log_replayer.perception_eval_conversions as eval_conversions

from driving_log_replayer_msgs.msg import GroundSegmentationEvalResult
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import ros2_numpy
import numpy as np

import glob
import tqdm

import json
import os


class GroundSegmentationEvaluator(DLREvaluator):
    CLOUD_DIM = 6

    def __init__(self, name: str) -> None:
        super().__init__(name, GroundSegmentationScenario, GroundSegmentationResult)

        # self.target_dataset_dir = "/home/toyozoshimada/perception_dataset/data/"
        # t4 datasetには~/driving_log_replayer_data/ground_segmentation_pcd/sample/t4_dataset/sample_dataset
        # self.target_pcd_file_path = "/home/toyozoshimada/perception_dataset/data/komatsu005_non_annotated_t4_format/komatsu005/data/LIDAR_CONCAT/*.bin"
        # self.get_logger().info(
        #     "========================================================================="
        # )
        # self.get_logger().info(f"{len(self._t4_dataset_paths)}")
        # for path in self._t4_dataset_paths:
        #     self.get_logger().info(path)

        # sample_data = json.load(
        #     open(os.path.join(self._t4_dataset_paths[0], "annotation", "sample_data.json"), "r")
        # )
        # sample_data = list(filter(lambda d: d["filename"].split(".")[-2] == "pcd", sample_data))

        # self.ground_truth = {}
        # for i in tqdm.tqdm(range(len(sample_data))):
        #     pcd_file_path = os.path.join(self._t4_dataset_paths[0], sample_data[i]["filename"])
        #     raw_points = np.fromfile(pcd_file_path, dtype=np.float32)
        #     points: np.ndarray = raw_points.reshape((-1, self.CLOUD_DIM))
        #     self.ground_truth[sample_data[i]["timestamp"]] = points

        self.__sub_pointcloud = self.create_subscription(
            PointCloud2,
            "/perception/obstacle_segmentation/single_frame/pointcloud_raw",
            self.pointcloud_cb,
            qos_profile_sensor_data,
        )

    def pointcloud_cb(self, msg: PointCloud2):
        # unix_time: int = eval_conversions.unix_time_from_ros_msg(msg.header)

        # min_time: int = abs(unix_time - list(self.ground_truth.keys())[0])
        # for i in range(1,len(self.ground_truth)):
        #     diff_time = abs(unix_time - list(self.ground_truth.keys())[i])
        #     if diff_time < min_time:
        #         min_time = diff_time

        # if min_time > 75000:
        #     self._logger.warn("time diff is too big")
        #     return

        # numpy_pcd = ros2_numpy.numpify(msg)
        # pointcloud = np.zeros((numpy_pcd.shape[0],3))
        # pointcloud[:, 0] = numpy_pcd["x"]
        # pointcloud[:, 1] = numpy_pcd["y"]
        # pointcloud[:, 2] = numpy_pcd["z"]

        self._logger.info("get numpy pointcloud")

        # for test
        tmp_msg = GroundSegmentationEvalResult()
        self._result.set_frame(tmp_msg)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return GroundSegmentationEvaluator("ground_segmentation_evaluator")


if __name__ == "__main__":
    main()
