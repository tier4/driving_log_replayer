#!/usr/bin/env python3

from driving_log_replayer.evaluator import DLREvaluator
from driving_log_replayer.evaluator import evaluator_main
from driving_log_replayer.ground_segmentation import GroundSegmentationResult
from driving_log_replayer.ground_segmentation import GroundSegmentationScenario

from driving_log_replayer_msgs.msg import GroundSegmentationEvalResult
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import ros2_numpy
import numpy as np


class GroundSegmentationEvaluator(DLREvaluator):
    def __init__(self, name: str) -> None:
        super().__init__(name, GroundSegmentationScenario, GroundSegmentationResult)

        eval_result_qos_policy = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.__sub_ground_seg_eval_result = self.create_subscription(
            GroundSegmentationEvalResult,
            "ground_segmentation/evaluation/result",
            self.eval_result_cb,
            eval_result_qos_policy,
        )

    def eval_result_cb(self, msg: GroundSegmentationEvalResult):
        self._result.set_frame(msg)
        self._result_writer.write_result(self._result)


@evaluator_main
def main() -> DLREvaluator:
    return GroundSegmentationEvaluator("ground_segmentation_evaluator")


if __name__ == "__main__":
    main()
