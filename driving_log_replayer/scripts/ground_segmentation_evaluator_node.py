#!/usr/bin/env python3

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.ground_segmentation import GroundSegmentationResult
from driving_log_replayer.ground_segmentation import GroundSegmentationScenario

from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import ros2_numpy
import numpy as np


class GroundSegmentationEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, GroundSegmentationScenario, GroundSegmentationResult)

        self.concat_pointcloud: PointCloud2 = None
        self.non_ground_pointcloud: PointCloud2 = None

        pointcloud_qos_policy = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.__sub_concat_pointcloud = self.create_subscription(
            PointCloud2,
            "/sensing/lidar/concatenated/pointcloud",
            self.concat_pointcloud_cb,
            pointcloud_qos_policy,
        )
        self.__sub_non_ground_point_cluod = self.create_subscription(
            PointCloud2,
            "/perception/obstacle_segmentation/single_frame/pointcloud_raw",
            self.non_ground_pointcloud_cb,
            pointcloud_qos_policy,
        )

    def concat_pointcloud_cb(self, msg: PointCloud2):
        self.concat_pointcloud = msg

    def non_ground_pointcloud_cb(self, msg: PointCloud2):
        self.non_ground_pointcloud = msg

        # if self.concat_pointcloud is not None:
        # numpy_gt_pcd = ros2_numpy.numpify(self.concat_pointcloud)
        # numpy_target_pcd = ros2_numpy.numpify(self.non_ground_pointcloud)
        # self._logger.info(f"obstacle cloud num : {len(numpy_gt_pcd)}")


@evaluator_main
def main() -> DLREvaluator:
    return GroundSegmentationEvaluator("ground_segmentation_evaluator")


if __name__ == "__main__":
    main()
