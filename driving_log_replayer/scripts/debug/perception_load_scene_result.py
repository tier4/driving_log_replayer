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
import os
import pickle

from perception_eval.config.perception_evaluation_config import PerceptionEvaluationConfig
from perception_eval.evaluation.metrics.metrics import MetricsScore
from perception_eval.manager.perception_evaluation_manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer3D
from perception_eval.util.logger_config import configure_logger
import yaml


class PerceptionEvaluatorPickle:
    def __init__(
        self, pickle_path: str, scenario_path: str, t4_dataset_path: str, log_path: str
    ) -> None:
        self.__scenario_yaml_obj = None
        self.__loaded_frame_results = None
        self.__t4_dataset_paths = [os.path.expandvars(t4_dataset_path)]
        self.__perception_eval_log_path = log_path
        with open(scenario_path) as scenario_file:
            self.__scenario_yaml_obj = yaml.safe_load(scenario_file)

        with open(pickle_path, "rb") as pickle_file:
            self.__loaded_frame_results = pickle.load(pickle_file)

        p_cfg = self.__scenario_yaml_obj["Evaluation"]["PerceptionEvaluationConfig"]

        evaluation_task = p_cfg["evaluation_config_dict"]["evaluation_task"]
        frame_id = self.__get_frame_id(evaluation_task)

        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=self.__t4_dataset_paths,
            frame_id=frame_id,
            merge_similar_labels=False,
            does_use_pointcloud=False,
            result_root_directory=os.path.join(self.__perception_eval_log_path, "result", "{TIME}"),
            evaluation_config_dict=p_cfg["evaluation_config_dict"],
        )
        _ = configure_logger(
            log_file_directory=evaluation_config.log_directory,
            console_log_level=logging.INFO,
            file_log_level=logging.INFO,
        )
        self.__evaluator = PerceptionEvaluationManager(evaluation_config=evaluation_config)
        self.__evaluator.frame_results = self.__loaded_frame_results

    def __get_frame_id(self, task: str) -> str:
        if task == "detection":
            frame_id = "base_link"
        elif task == "tracking":
            frame_id = "map"
        else:
            error_msg = f"Unexpected evaluation task: {task}"
            raise ValueError(error_msg)  # EM102
        return frame_id

    def get_final_result(self) -> MetricsScore:
        # use case fail object num
        number_use_case_fail_object: int = 0
        for frame_results in self.__evaluator.frame_results:
            number_use_case_fail_object += frame_results.pass_fail_result.get_fail_object_num()
        logging.info(f"final use case fail object: {number_use_case_fail_object}")
        final_metric_score = self.__evaluator.get_scene_result()
        logging.info(f"final metrics result {final_metric_score}")
        analyzer = PerceptionAnalyzer3D(self.__evaluator.evaluator_config)
        analyzer.add(self.__evaluator.frame_results)
        score_df, error_df = analyzer.analyze()
        if score_df is not None:
            score_dict = score_df.to_dict()
        if error_df is not None:
            error_dict = (
                error_df.groupby(level=0).apply(lambda df: df.xs(df.name).to_dict()).to_dict()
            )
        print(score_dict)
        print(error_dict)
        return final_metric_score


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p", "--pickle", required=True, help="path of the pickle file to load scene_result"
    )
    parser.add_argument(
        "-s", "--scenario", required=True, help="path of the scenario to load evaluator settings"
    )
    parser.add_argument("-d", "--dataset", required=True, help="path of the t4_dataset")
    parser.add_argument("-l", "--log_path", required=True, help="perception_eval log path")
    args = parser.parse_args()
    evaluator = PerceptionEvaluatorPickle(
        os.path.expandvars(args.pickle),
        os.path.expandvars(args.scenario),
        os.path.expandvars(args.dataset),
        os.path.expandvars(args.log_path),
    )
    evaluator.get_final_result()


if __name__ == "__main__":
    main()
