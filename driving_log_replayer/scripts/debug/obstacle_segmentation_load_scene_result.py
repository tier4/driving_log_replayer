#!/usr/bin/env python3

# Copyright (c) 2021 TIER IV.inc
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

import argparse
import logging
from os.path import expandvars
from pathlib import Path
import pickle

import numpy as np
from perception_eval.config.sensing_evaluation_config import SensingEvaluationConfig
from perception_eval.manager.sensing_evaluation_manager import SensingEvaluationManager
from perception_eval.util.logger_config import configure_logger
import yaml


class ObstacleSegmentationEvaluatorPickle:
    def __init__(
        self,
        pickle_path: str,
        scenario_path: str,
        t4_dataset_path: str,
        log_path: str,
    ) -> None:
        self.__scenario_yaml_obj = None
        self.__loaded_frame_results = None
        self.__t4_dataset_paths = [expandvars(t4_dataset_path)]
        self.__perception_eval_log_path = Path(log_path)
        with Path(scenario_path).open() as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)

        with Path(pickle_path).open("rb") as pickle_file:
            self.__loaded_frame_results = pickle.load(pickle_file)

        s_cfg = self.__scenario_yaml_obj["Evaluation"]["SensingEvaluationConfig"]

        evaluation_config: SensingEvaluationConfig = SensingEvaluationConfig(
            dataset_paths=self.__t4_dataset_paths,
            frame_id="base_link",
            merge_similar_labels=False,
            does_use_pointcloud=False,
            result_root_directory=self.__perception_eval_log_path.joinpath(
                "result",
                "{TIME}",
            ).as_posix(),
            evaluation_config_dict=s_cfg["evaluation_config_dict"],
        )
        _ = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
        )
        self.__evaluator = SensingEvaluationManager(evaluation_config=evaluation_config)
        self.__evaluator.frame_results = self.__loaded_frame_results
        self.debug_frames()

    def debug_frames(self) -> None:
        for result in self.__evaluator.frame_results:
            print(f"FrameName: {result.frame_name}")  # noqa
            dist_array = np.array([])
            for pcd in result.pointcloud_failed_non_detection:
                dists: np.ndarray = np.linalg.norm(pcd, ord=2, axis=1)
                print(dists)  # noqa
                dist_array = np.concatenate([dist_array, dists])
            print(dist_array)  # noqa

    def get_final_result(self) -> None:
        """Output the evaluation results on the command line."""
        # use case fail object num
        num_use_case_fail: int = 0
        for frame_results in self.__evaluator.frame_results:
            num_use_case_fail += len(frame_results.detection_fail_results)
        logging.warning("%d fail results.", num_use_case_fail)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--pickle",
        required=True,
        help="path of the pickle file to load scene_result",
    )
    parser.add_argument(
        "-s",
        "--scenario",
        required=True,
        help="path of the scenario to load evaluator settings",
    )
    parser.add_argument("-d", "--dataset", required=True, help="path of the t4_dataset")
    parser.add_argument("-l", "--log_path", required=True, help="perception_eval log path")
    args = parser.parse_args()
    evaluator = ObstacleSegmentationEvaluatorPickle(
        expandvars(args.pickle),
        expandvars(args.scenario),
        expandvars(args.dataset),
        expandvars(args.log_path),
    )
    evaluator.get_final_result()


if __name__ == "__main__":
    main()
